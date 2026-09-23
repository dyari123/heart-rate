/*
  Heart Monitor - Arduino Nano + MAX30102 + 0.96" SSD1306 OLED + buzzer

  Wiring (everything on the same I2C bus):
    MAX30102  VIN -> 5V, GND -> GND, SDA -> A4, SCL -> A5
    OLED      VCC -> 5V, GND -> GND, SDA -> A4, SCL -> A5
    Buzzer    +   -> D8, -   -> GND

  Libraries (Arduino Library Manager):
    "SparkFun MAX3010x Pulse and Proximity Sensor Library"
    "U8g2" by oliver

  Serial Monitor: 115200 baud. One line per second from 1 s to 60 s,
  then a summary of the whole minute.

  This is a hobby sensor, NOT a medical device.
*/

#include <Wire.h>
#include "MAX30105.h"
#include <U8g2lib.h>

// =====================================================
// PINS / DISPLAY
// =====================================================

#define BUZZER_PIN 8

// 1 = print the filtered pulse signal on every sample, for the Arduino
// Serial Plotter. Handy for checking the signal and tuning the thresholds.
#define PLOT_SIGNAL 0

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
MAX30105 sensor;

// =====================================================
// SETTINGS
// =====================================================

// Sensor: 400 samples/s, averaged 4 at a time inside the MAX30102, so we get
// 100 clean samples per second (one every 10 ms).
const byte LED_POWER = 0x1F;          // starting LED current (~6.4 mA); auto-tuned per finger
const byte SAMPLE_AVERAGE = 4;
const int SENSOR_SAMPLE_RATE = 400;
const int ADC_RANGE = 4096;           // use 8192 or 16384 if IR is still near 262143 at low current
const unsigned int SAMPLE_MS = 10;    // 1000 / (400 / 4)

// Sensor clock correction = "sensor ms / real ms" from the summary,
// averaged over 3-4 runs. Leave at 1.000 if they differ by less than 0.5%.
const float CLOCK_FIX = 1.000;

// LED auto-tune: aim for a raw level of 100,000 - 200,000 (ADC max 262,143).
const long DC_LOW = 100000;
const long DC_HIGH = 200000;
const long DC_TARGET = 150000;
const long SATURATION_LEVEL = 260000; // above this the pulse top is clipped
const byte LED_MIN = 0x04;
const byte LED_MAX = 0xF0;
const byte MAX_TUNE_STEPS = 6;        // per measurement

// Finger detection (at the starting LED current). The Serial Monitor prints
// "IR=" every second while it waits; set these well above the no-finger value.
const long FINGER_ON_LEVEL = 20000;
const long FINGER_OFF_LEVEL = 12000;  // scaled automatically when the LED current changes
const byte FINGER_ON_SAMPLES = 10;    // 0.10 s above ON level before we start
const byte FINGER_OFF_SAMPLES = 25;   // 0.25 s below OFF level before we give up

// Beat detection
const unsigned int SETTLE_MS = 1500;          // ignore the first moments while the finger settles
const float DC_ALPHA = 0.02;                  // slow baseline (removes drift / finger pressure)
const float MIN_HYSTERESIS = 40;              // smallest swing that counts as a turn in the wave
const float MIN_PULSE_AMPLITUDE = 80;         // smallest valley-to-peak that counts as a beat
const float MIN_PERFUSION = 0.2;              // % (pulse / DC). Weaker beats can't be timed reliably
const float MOTION_LIMIT = 0.02;              // baseline jump of 2% within 0.5 s = finger moved
const unsigned int MOTION_PAUSE_MS = 1000;
const unsigned int MIN_BEAT_INTERVAL = 300;   // 200 BPM
const unsigned int MAX_BEAT_INTERVAL = 1500;  // 40 BPM
const unsigned int SIGNAL_LOST_MS = 3000;     // no beat for this long -> show "--" and re-learn
const float INTERVAL_TOLERANCE = 0.25;        // beat interval may differ 25% from the median
const byte REJECT_WARN_PERCENT = 15;          // above this: "irregular or noisy signal"

// SpO2 = SPO2_A - SPO2_B * R   (R = ratio of Red to IR pulse strength)
// Calibrate: SPO2_A = oximeterSpO2 + 25 * R (R = "Average R" in the summary),
// averaged over 10+ sessions. See the README.
const float SPO2_A = 110.0;
const float SPO2_B = 25.0;
const byte SPO2_MIN_BEATS = 5;                // good beats needed before SpO2 is shown

// Measurement
const unsigned long HALF_WINDOW = 30000;
const unsigned long MEASURE_WINDOW = 60000;
const unsigned long DISPLAY_INTERVAL = 100;   // screen animation (numbers change once per second)

// MAX30102 registers used for direct FIFO reading
const byte MAX_ADDR = 0x57;
const byte REG_OVF_COUNTER = 0x05;
const byte REG_FIFO_DATA = 0x07;

// =====================================================
// STATE
// =====================================================

// Finger / measurement
bool fingerPresent = false;
byte fingerOnCount = 0;
byte fingerOffCount = 0;
bool welcomeDrawn = false;

unsigned long measurementStart = 0;
bool halfDone = false;
bool measurementDone = false;
int average30 = 0;
int average60 = 0;
unsigned long intervalSumA = 0;   // intervals in 0-30 s
unsigned int intervalCountA = 0;
unsigned long intervalSumAll = 0; // intervals in 0-60 s
unsigned int intervalCountAll = 0;

// Quality counters for the 60 s summary
unsigned int beatsAccepted = 0;
unsigned int beatsRejected = 0;
unsigned int intervalsRejected = 0;
unsigned int motionEvents = 0;

// Values shown on screen / Serial, refreshed exactly once per second
unsigned long lastSecond = 0;
int shownBPM = 0;
int shownSpO2 = 0;
int minBPM = 0, maxBPM = 0;
long spo2Sum = 0;
unsigned int spo2Seconds = 0;
unsigned int bpmSeconds = 0;
float ratioSum = 0;

int currentSpO2 = 0;
float lastPerfusion = 0;

// Time base built from the sensor's own sample rate, so beat timing stays
// exact even while the display is busy.
unsigned long sampleClock = 0;
unsigned long sampleClockStart = 0;
unsigned long samplesUsed = 0;
unsigned long samplesLost = 0;
uint32_t lastIR = 0;

// LED currents (tuned separately; the SpO2 math divides each by its own DC)
byte irPower = LED_POWER;
byte redPower = LED_POWER;
bool ledDirty = false;
byte tuneSteps = 0;
byte tuneTick = 0;
byte tuneHold = 0;
byte clipRun = 0;

// Filters
bool filtersReady = false;
float irDC = 0, redDC = 0;   // baselines
float irAC = 0, redAC = 0;   // pulse signal (baseline removed, filtered, flipped so beats point up)
float irLP[4], redLP[4];     // Butterworth filter memory {x1, x2, y1, y2}
unsigned long settleUntil = 0;

// Motion check: baseline history, one value every 100 ms (0.5 s total)
#define DC_HIST 5
float dcHist[DC_HIST];
byte dcHistHead = 0;
byte motionTick = 0;
unsigned long motionUntil = 0;
bool inMotion = false;

unsigned long lastClipTime = 0;
unsigned long lastWeakTime = 0;

// Peak / valley detector (with hysteresis)
bool lookingForPeak = true;
bool haveValley = false;
float extremeIR = 0;
float valleyIR = 0;
float extremeRed = 0, valleyRed = 0;   // Red taken at the same moments as the IR peak / valley
float pulseAmplitude = 0;    // learned typical beat size (0 = still learning)
unsigned long lastBeatTime = 0;
unsigned long lastActivity = 0;

// Beat timing point = the steepest moment of the pulse upstroke. It is much
// sharper than the rounded top of the wave, so beat-to-beat timing is precise.
float prevIrAC = 0;
float prevSlope = 0;
float maxSlope = 0;
float slopeBeforeMax = 0, slopeAfterMax = 0;
unsigned long maxSlopeTime = 0;
bool needSlopeAfter = false;

// Last beat-to-beat intervals (ms)
#define IBI_SIZE 8
unsigned int ibi[IBI_SIZE];
byte ibiCount = 0;
byte ibiHead = 0;
byte outlierStreak = 0;

// SpO2 ratio of the last beats (median is used, so one bad beat can't move it)
#define RATIO_SIZE 8
float ratios[RATIO_SIZE];
byte ratioCount = 0;
byte ratioHead = 0;
float medianRatio = 0;

// Screen effects
bool heartFlash = false;
unsigned long heartFlashStart = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastWaitPrint = 0;
unsigned long lastWaveUpdate = 0;

// Fake ECG waveform
#define WAVE_SAMPLES 64
int8_t ecgWave[WAVE_SAMPLES];
int spikeStep = -1;
float wavePhase = 0;

const int8_t spikePattern[] PROGMEM = {0, -2, 0, 2, -8, 4, -2, 0};
const int SPIKE_LEN = sizeof(spikePattern);

// =====================================================
// SMALL HELPERS
// =====================================================

// Sort a small array in place (insertion sort, fine for 8 values).
void sortUInt(unsigned int *a, byte n) {
  for (byte i = 1; i < n; i++) {
    unsigned int v = a[i];
    byte j = i;
    while (j > 0 && a[j - 1] > v) {
      a[j] = a[j - 1];
      j--;
    }
    a[j] = v;
  }
}

void sortFloat(float *a, byte n) {
  for (byte i = 1; i < n; i++) {
    float v = a[i];
    byte j = i;
    while (j > 0 && a[j - 1] > v) {
      a[j] = a[j - 1];
      j--;
    }
    a[j] = v;
  }
}

// Append a number to a text buffer (smaller than sprintf on the Nano).
void appendInt(char *buf, long v) {
  char tmp[12];
  ltoa(v, tmp, 10);
  strcat(buf, tmp);
}

// 2nd-order Butterworth low-pass, 5 Hz at 100 samples/s. Much steeper than
// simple smoothing, and its delay is constant, so beat intervals are unchanged.
// s[] = {x1, x2, y1, y2}
float lowPass(float x, float *s) {
  float y = 0.0200834 * x + 0.0401667 * s[0] + 0.0200834 * s[1]
          + 1.5610181 * s[2] - 0.6413515 * s[3];
  s[1] = s[0];
  s[0] = x;
  s[3] = s[2];
  s[2] = y;
  return y;
}

// =====================================================
// RESET / START
// =====================================================

void clearRatios() {
  ratioCount = 0;
  ratioHead = 0;
  medianRatio = 0;
  currentSpO2 = 0;
}

void clearIntervals() {
  ibiCount = 0;
  ibiHead = 0;
  outlierStreak = 0;
}

// Start looking for beats from scratch without trusting the last beat time
// (used after motion, lost samples or an LED change).
void resetBeatChain() {
  lookingForPeak = true;
  haveValley = false;
  extremeIR = irAC;
  extremeRed = redAC;
  maxSlope = 0;
  needSlopeAfter = false;
  lastBeatTime = 0;
}

void resetDetector() {
  filtersReady = false;
  irDC = redDC = 0;
  irAC = redAC = 0;

  resetBeatChain();
  valleyIR = 0;
  valleyRed = 0;
  pulseAmplitude = 0;
  lastActivity = sampleClock;

  prevIrAC = 0;
  prevSlope = 0;
  slopeBeforeMax = slopeAfterMax = 0;
  maxSlopeTime = 0;

  motionUntil = 0;
  inMotion = false;
  lastClipTime = 0;
  lastWeakTime = 0;
  lastPerfusion = 0;
  clipRun = 0;

  clearIntervals();
  clearRatios();
}

void resetMeasurement() {
  fingerPresent = false;
  welcomeDrawn = false;
  fingerOnCount = 0;
  fingerOffCount = 0;
  measurementStart = 0;
  halfDone = false;
  measurementDone = false;
  average30 = 0;
  average60 = 0;
  intervalSumA = 0;
  intervalCountA = 0;
  intervalSumAll = 0;
  intervalCountAll = 0;

  beatsAccepted = 0;
  beatsRejected = 0;
  intervalsRejected = 0;
  motionEvents = 0;

  lastSecond = 0;
  shownBPM = 0;
  shownSpO2 = 0;
  minBPM = maxBPM = 0;
  spo2Sum = 0;
  spo2Seconds = 0;
  bpmSeconds = 0;
  ratioSum = 0;

  // Back to the starting LED current so the finger thresholds are valid.
  if (irPower != LED_POWER || redPower != LED_POWER) {
    irPower = redPower = LED_POWER;
    ledDirty = true;
  }
  tuneSteps = 0;
  tuneTick = 0;
  tuneHold = 0;

  heartFlash = false;
  noTone(BUZZER_PIN);
  resetDetector();
  for (int i = 0; i < WAVE_SAMPLES; i++) ecgWave[i] = 0;
  spikeStep = -1;
}

void startMeasurement() {
  resetMeasurement();
  fingerPresent = true;
  measurementStart = millis();
  samplesUsed = 0;
  samplesLost = 0;
  Serial.println();
  Serial.println(F("FINGER DETECTED - 60 s MEASUREMENT STARTED"));
  Serial.println(F("time   BPM   SpO2   R       PI"));
  sampleClockStart = sampleClock;
}

// Heart rhythm lost (moved finger, pressed too hard...). Show "--" instead of
// freezing on an old number, and re-learn the pulse size from scratch.
void signalLost() {
  pulseAmplitude = 0;
  lastBeatTime = 0;
  lastActivity = sampleClock;
  clearIntervals();
  clearRatios();
}

// =====================================================
// LED AUTO-TUNE (only while the signal settles)
// =====================================================

bool tuneOne(uint32_t level, byte &power) {
  if ((long)level >= DC_LOW && (long)level <= DC_HIGH) return false;
  long p = (long)power * DC_TARGET / (long)(level > 0 ? level : 1);
  if (p < LED_MIN) p = LED_MIN;
  if (p > LED_MAX) p = LED_MAX;
  if (p == power) return false;
  power = (byte)p;
  return true;
}

// Returns true if an LED current was changed. The new current is sent to the
// sensor from pumpSensor(), never in the middle of reading a FIFO block.
bool tuneLeds(uint32_t ir, uint32_t red) {
  if (tuneSteps >= MAX_TUNE_STEPS) return false;
  bool changed = tuneOne(ir, irPower);
  if (tuneOne(red, redPower)) changed = true;
  if (!changed) return false;

  tuneSteps++;
  ledDirty = true;
  tuneHold = 8;           // skip samples that were taken with the old current
  filtersReady = false;   // restart baseline + settle time at the new level
  return true;
}

void applyLeds() {
  if (!ledDirty) return;
  ledDirty = false;
  sensor.setPulseAmplitudeIR(irPower);
  sensor.setPulseAmplitudeRed(redPower);
}

// =====================================================
// DRAW HEART ICONS
// =====================================================

void drawHeartSmall(int x, int y, bool filled) {
  if (filled) {
    u8g2.drawBox(x + 2, y, 4, 4);
    u8g2.drawBox(x + 8, y, 4, 4);
    u8g2.drawBox(x, y + 3, 14, 5);
    u8g2.drawBox(x + 2, y + 8, 10, 4);
    u8g2.drawBox(x + 5, y + 12, 4, 3);
  } else {
    u8g2.drawPixel(x + 2, y + 1);
    u8g2.drawPixel(x + 3, y);
    u8g2.drawPixel(x + 5, y + 1);
    u8g2.drawPixel(x + 8, y + 1);
    u8g2.drawPixel(x + 9, y);
    u8g2.drawPixel(x + 11, y + 1);
    u8g2.drawPixel(x + 1, y + 4);
    u8g2.drawPixel(x + 2, y + 5);
    u8g2.drawPixel(x + 3, y + 6);
    u8g2.drawPixel(x + 4, y + 7);
    u8g2.drawPixel(x + 5, y + 8);
    u8g2.drawPixel(x + 6, y + 9);
    u8g2.drawPixel(x + 7, y + 8);
    u8g2.drawPixel(x + 8, y + 7);
    u8g2.drawPixel(x + 9, y + 6);
    u8g2.drawPixel(x + 10, y + 5);
    u8g2.drawPixel(x + 11, y + 4);
  }
}

void drawHeartBig(int x, int y) {
  u8g2.drawBox(x + 4, y, 8, 8);
  u8g2.drawBox(x + 16, y, 8, 8);
  u8g2.drawBox(x, y + 6, 28, 10);
  u8g2.drawBox(x + 4, y + 16, 20, 8);
  u8g2.drawBox(x + 10, y + 24, 8, 6);
}

// =====================================================
// BEAT FEEDBACK / SpO2
// =====================================================

void beatFeedback() {
  tone(BUZZER_PIN, 1568, 70);
  heartFlash = true;
  heartFlashStart = millis();
  spikeStep = 0;
}

void updateSpO2(float irAmp, float redAmp) {
  if (irAmp <= 0 || redAmp <= 0 || irDC <= 0 || redDC <= 0) return;

  float ratio = (redAmp / redDC) / (irAmp / irDC);
  if (ratio < 0.1 || ratio > 2.0) return;   // nonsense beat, don't let it pull the result

  ratios[ratioHead] = ratio;
  ratioHead = (ratioHead + 1) % RATIO_SIZE;
  if (ratioCount < RATIO_SIZE) ratioCount++;

  if (ratioCount >= SPO2_MIN_BEATS) {
    float sorted[RATIO_SIZE];
    for (byte i = 0; i < ratioCount; i++) sorted[i] = ratios[i];
    sortFloat(sorted, ratioCount);
    medianRatio = sorted[ratioCount / 2];

    float spo2f = SPO2_A - SPO2_B * medianRatio;
    if (spo2f > 100) spo2f = 100;
    if (spo2f < 70) spo2f = 70;
    currentSpO2 = (int)(spo2f + 0.5);
  }
}

// =====================================================
// BEAT INTERVALS -> BPM
// =====================================================

unsigned int medianInterval() {
  unsigned int sorted[IBI_SIZE];
  for (byte i = 0; i < ibiCount; i++) sorted[i] = ibi[i];
  sortUInt(sorted, ibiCount);
  return sorted[ibiCount / 2];
}

// Average BPM from a total of intervals, with the sensor clock correction.
int bpmFrom(unsigned long intervalSum, unsigned int count) {
  if (intervalSum == 0) return 0;
  return (int)(60000.0 * count / intervalSum * CLOCK_FIX + 0.5);
}

// Heart rate from the last (up to) 8 good beat intervals. With 5 or more,
// the shortest and longest are dropped first (trimmed mean), so one slightly
// mistimed beat can't pull the number.
int computeBPM() {
  if (ibiCount == 0) return 0;

  unsigned int sorted[IBI_SIZE];
  for (byte i = 0; i < ibiCount; i++) sorted[i] = ibi[i];
  sortUInt(sorted, ibiCount);

  byte first = 0, last = ibiCount;
  if (ibiCount >= 5) {
    first = 1;
    last = ibiCount - 1;
  }
  unsigned long sum = 0;
  for (byte i = first; i < last; i++) sum += sorted[i];
  return bpmFrom(sum, last - first);
}

void addInterval(unsigned int interval) {
  // Reject a single odd interval (missed or extra beat), but if the rhythm
  // really changed, 3 odd ones in a row start a fresh history.
  if (ibiCount >= 3) {
    unsigned int med = medianInterval();
    unsigned int diff = (interval > med) ? interval - med : med - interval;
    if (diff > med * INTERVAL_TOLERANCE) {
      outlierStreak++;
      if (outlierStreak < 3) {
        if (!measurementDone) intervalsRejected++;
        return;
      }
      clearIntervals();
    }
  }
  outlierStreak = 0;

  ibi[ibiHead] = interval;
  ibiHead = (ibiHead + 1) % IBI_SIZE;
  if (ibiCount < IBI_SIZE) ibiCount++;

  // Official 30 s / 60 s averages only use intervals that passed every check.
  if (!measurementDone) {
    unsigned long elapsed = millis() - measurementStart;
    if (elapsed < HALF_WINDOW) {
      intervalSumA += interval;
      intervalCountA++;
    }
    if (elapsed < MEASURE_WINDOW) {
      intervalSumAll += interval;
      intervalCountAll++;
    }
  }
}

void rejectBeat() {
  if (!measurementDone) beatsRejected++;
}

// A full pulse was found in the IR signal. beatTime is the precise moment of
// its steepest upstroke.
void onPulse(float amplitude, float redAmplitude, unsigned long beatTime) {
  if (amplitude < MIN_PULSE_AMPLITUDE) return;   // noise, not a beat

  // Second bump of the same heartbeat (dicrotic notch) or noise. Once the
  // rhythm is known, anything sooner than 60% of the usual interval is ignored.
  if (lastBeatTime != 0) {
    unsigned long since = beatTime - lastBeatTime;
    if (since < MIN_BEAT_INTERVAL) return;
    if (ibiCount >= 3 && since < medianInterval() * 0.6) return;
  }

  // Pulse top was clipped by the ADC: the timing can't be trusted.
  if (lastClipTime != 0 && sampleClock - lastClipTime < 1000) {
    rejectBeat();
    return;
  }

  // Perfusion index: how big the pulse is compared to the steady light level.
  lastPerfusion = amplitude / irDC * 100.0;
  if (lastPerfusion < MIN_PERFUSION) {
    lastWeakTime = sampleClock;
    rejectBeat();
    return;
  }

  // Much bigger or smaller than the usual beat = finger movement.
  if (pulseAmplitude > 0 &&
      (amplitude > pulseAmplitude * 3 || amplitude < pulseAmplitude * 0.3)) {
    rejectBeat();
    return;
  }

  pulseAmplitude = (pulseAmplitude == 0) ? amplitude : pulseAmplitude * 0.8 + amplitude * 0.2;
  lastActivity = sampleClock;
  if (!measurementDone) beatsAccepted++;

  beatFeedback();
  updateSpO2(amplitude, redAmplitude);

  if (lastBeatTime == 0) {
    lastBeatTime = beatTime;
    return;
  }

  unsigned long interval = beatTime - lastBeatTime;
  lastBeatTime = beatTime;

  // A gap of about 2 or 3 normal beats means beats in between were missed or
  // rejected. Split it into the missed beats instead of throwing it away
  // (or worse, counting it as one very slow beat).
  if (ibiCount > 0) {
    unsigned int reference = medianInterval();
    unsigned long beats = (interval + reference / 2) / reference;
    if (beats >= 2 && beats <= 3) {
      unsigned int part = (unsigned int)(interval / beats);
      unsigned int diff = (part > reference) ? part - reference : reference - part;
      if (diff <= reference * INTERVAL_TOLERANCE) {
        addInterval(part);
        return;
      }
    }
  } else if (interval > MAX_BEAT_INTERVAL && interval <= 2UL * MAX_BEAT_INTERVAL) {
    addInterval((unsigned int)(interval / 2));   // no history yet: assume one missed beat
    return;
  }

  if (interval <= MAX_BEAT_INTERVAL) {
    addInterval((unsigned int)interval);
  } else if (!measurementDone) {
    intervalsRejected++;
  }
}

// =====================================================
// SIGNAL PROCESSING (called for EVERY sensor sample)
// =====================================================

void processSample(uint32_t ir, uint32_t red) {
  if (tuneHold > 0) {
    tuneHold--;
    return;
  }

  // Clipping check. If it lasts 0.5 s during the measurement, re-tune the LEDs.
  if ((long)ir > SATURATION_LEVEL || (long)red > SATURATION_LEVEL) {
    lastClipTime = sampleClock;
    if (++clipRun >= 50 && filtersReady && (long)(sampleClock - settleUntil) >= 0 &&
        tuneSteps < MAX_TUNE_STEPS) {
      clipRun = 0;
      tuneSteps++;
      filtersReady = false;   // settle again, where the LEDs get re-tuned
    }
  } else {
    clipRun = 0;
  }

  if (!filtersReady) {
    irDC = ir;
    redDC = red;
    irAC = 0;
    redAC = 0;
    prevIrAC = 0;
    for (byte i = 0; i < 4; i++) irLP[i] = redLP[i] = 0;
    for (byte i = 0; i < DC_HIST; i++) dcHist[i] = ir;
    dcHistHead = 0;
    motionTick = 0;
    tuneTick = 0;
    resetBeatChain();
    filtersReady = true;
    settleUntil = sampleClock + SETTLE_MS;
    lastActivity = settleUntil;
    return;
  }

  irDC += DC_ALPHA * ((float)ir - irDC);
  redDC += DC_ALPHA * ((float)red - redDC);

  // Blood absorbs light, so every heartbeat makes the reading DROP.
  // Flip the sign so a heartbeat is an upward peak, then low-pass filter.
  irAC = lowPass(irDC - (float)ir, irLP);
  redAC = lowPass(redDC - (float)red, redLP);

  float slope = irAC - prevIrAC;
  prevIrAC = irAC;

#if PLOT_SIGNAL
  Serial.println(irAC);
#endif

  // Baseline history for the motion check: one value every 100 ms (0.5 s kept).
  bool motion = false;
  if (++motionTick >= 10) {
    motionTick = 0;
    float old = dcHist[dcHistHead];
    dcHist[dcHistHead] = irDC;
    dcHistHead = (dcHistHead + 1) % DC_HIST;
    motion = fabs(irDC - old) > MOTION_LIMIT * irDC;
  }

  // Settle phase: tune the LED currents (every 200 ms), no beat detection.
  if ((long)(sampleClock - settleUntil) < 0) {
    prevSlope = slope;
    if (++tuneTick >= 20) {
      tuneTick = 0;
      tuneLeds(ir, red);
    }
    return;
  }

  if ((long)(sampleClock - lastActivity) > (long)SIGNAL_LOST_MS) signalLost();

  // Finger moved (baseline jumped more than 2% within 0.5 s): pause 1 s.
  if (motion) {
    if (!inMotion && !measurementDone) motionEvents++;
    inMotion = true;
    motionUntil = sampleClock + MOTION_PAUSE_MS;
    resetBeatChain();
  }
  if ((long)(sampleClock - motionUntil) < 0) {
    prevSlope = slope;
    lastActivity = sampleClock;   // a motion pause is not "no pulse"
    return;
  }
  if (inMotion) {
    // Restart the baseline at the new finger position, so the leftover step
    // from the move doesn't distort the next beats.
    inMotion = false;
    irDC = ir;
    redDC = red;
    irAC = redAC = prevIrAC = 0;
    for (byte i = 0; i < 4; i++) irLP[i] = redLP[i] = 0;
    for (byte i = 0; i < DC_HIST; i++) dcHist[i] = ir;
    resetBeatChain();
    prevSlope = 0;
    return;
  }

  // Track the steepest rise since the last valley (the pulse upstroke).
  if (needSlopeAfter) {
    slopeAfterMax = slope;
    needSlopeAfter = false;
  }
  if (slope > maxSlope) {
    maxSlope = slope;
    slopeBeforeMax = prevSlope;
    maxSlopeTime = sampleClock;
    needSlopeAfter = true;
  }
  prevSlope = slope;

  // Hysteresis: the wave must turn by a real amount (30% of a typical beat)
  // before we call it a peak or valley. Small wiggles on the slope and the
  // dicrotic notch are ignored instead of splitting one beat into pieces.
  float hysteresis = pulseAmplitude * 0.3;
  if (hysteresis < MIN_HYSTERESIS) hysteresis = MIN_HYSTERESIS;

  // Red is read at the exact moments of the IR peak and valley. Taking Red's
  // own highest/lowest point would add its noise on top and make SpO2 read low.
  if (lookingForPeak) {
    if (irAC >= extremeIR) {
      extremeIR = irAC;
      extremeRed = redAC;
    } else if (extremeIR - irAC > hysteresis) {
      float amplitude = extremeIR - valleyIR;
      float redAmplitude = extremeRed - valleyRed;

      // Fit a parabola through the slopes around the steepest point to find
      // it between two samples (about 1 ms precision instead of 10 ms).
      float offset = 0;
      float denom = slopeBeforeMax - 2 * maxSlope + slopeAfterMax;
      if (denom < 0) {
        offset = 0.5 * (slopeBeforeMax - slopeAfterMax) / denom;
        if (offset > 0.5) offset = 0.5;
        if (offset < -0.5) offset = -0.5;
      }
      unsigned long beatTime = maxSlopeTime + (long)(offset * SAMPLE_MS + (offset >= 0 ? 0.5 : -0.5));

      lookingForPeak = false;
      extremeIR = irAC;
      extremeRed = redAC;

      if (haveValley) onPulse(amplitude, redAmplitude, beatTime);
    }
  } else {
    if (irAC <= extremeIR) {
      extremeIR = irAC;
      extremeRed = redAC;
      maxSlope = 0;            // new lowest point: the upstroke starts after this
      needSlopeAfter = false;
    } else if (irAC - extremeIR > hysteresis) {
      valleyIR = extremeIR;
      valleyRed = extremeRed;
      haveValley = true;

      lookingForPeak = true;
      extremeIR = irAC;
      extremeRed = redAC;
    }
  }
}

void handleSample(uint32_t ir, uint32_t red) {
  sampleClock += SAMPLE_MS;
  samplesUsed++;
  lastIR = ir;

  if (!fingerPresent) {
    if ((long)ir > FINGER_ON_LEVEL) {
      if (++fingerOnCount >= FINGER_ON_SAMPLES) startMeasurement();
    } else {
      fingerOnCount = 0;
    }
    return;
  }

  // The no-finger level scales with the IR LED current.
  long offLevel = FINGER_OFF_LEVEL * (long)irPower / LED_POWER;
  if ((long)ir < offLevel) {
    if (++fingerOffCount >= FINGER_OFF_SAMPLES) {
      Serial.println(F("FINGER REMOVED"));
      resetMeasurement();
    }
    return;
  }
  fingerOffCount = 0;

  processSample(ir, red);
}

// =====================================================
// SENSOR READING
// =====================================================

uint32_t readValue18() {
  uint32_t v = Wire.read();
  v = (v << 8) | Wire.read();
  v = (v << 8) | Wire.read();
  return v & 0x3FFFF;
}

// Read EVERY sample waiting in the MAX30102's own 32-sample FIFO. We read it
// directly instead of through the library, whose buffer only holds 4 samples
// on a Nano and silently drops the rest. The chip's overflow counter tells us
// if samples were ever lost (printed in the summary; it should stay 0).
void pumpSensor() {
  byte lost = sensor.readRegister8(MAX_ADDR, REG_OVF_COUNTER);
  byte n = (sensor.getWritePointer() - sensor.getReadPointer()) & 0x1F;
  if (lost > 0) {
    samplesLost += lost;
    sampleClock += (unsigned long)lost * SAMPLE_MS;   // keep the time base correct
    if (filtersReady) resetBeatChain();                // the gap breaks the current beat
    if (n == 0) n = 32;                                // FIFO completely full
  }

  while (n > 0) {
    byte take = (n > 5) ? 5 : n;                       // 5 samples x 6 bytes fit the 32-byte Wire buffer
    Wire.beginTransmission(MAX_ADDR);
    Wire.write(REG_FIFO_DATA);
    Wire.endTransmission();
    Wire.requestFrom(MAX_ADDR, (byte)(take * 6));
    for (byte i = 0; i < take; i++) {
      uint32_t red = readValue18();
      uint32_t ir = readValue18();
      handleSample(ir, red);
    }
    n -= take;
  }

  applyLeds();
}

// =====================================================
// ONCE-PER-SECOND UPDATE (screen numbers + Serial line)
// =====================================================

// Short status word for the screen and Serial ("" = signal is fine).
const char *signalStatus() {
  if (!filtersReady || (long)(sampleClock - settleUntil) < 0) return "WAIT";
  if ((long)(sampleClock - motionUntil) < 0) return "MOTION";
  if (lastClipTime != 0 && sampleClock - lastClipTime < 1000) return "CLIPPED";
  if (lastWeakTime != 0 && sampleClock - lastWeakTime < 2000) return "WEAK";
  return "";
}

void updateEverySecond(unsigned long second) {
  shownBPM = computeBPM();
  shownSpO2 = currentSpO2;

  if (second > 60) return;   // after the minute: screen stays live, Serial stays quiet

  if (shownBPM > 0) {
    if (bpmSeconds == 0 || shownBPM < minBPM) minBPM = shownBPM;
    if (bpmSeconds == 0 || shownBPM > maxBPM) maxBPM = shownBPM;
    bpmSeconds++;
  }
  if (shownSpO2 > 0) {
    spo2Sum += shownSpO2;
    ratioSum += medianRatio;
    spo2Seconds++;
  }

#if !PLOT_SIGNAL
  if (second < 10) Serial.print(' ');
  Serial.print(second);
  Serial.print(F("s    "));
  if (shownBPM > 0) {
    if (shownBPM < 100) Serial.print(' ');
    Serial.print(shownBPM);
  } else {
    Serial.print(F(" --"));
  }
  Serial.print(F("   "));
  if (shownSpO2 > 0) {
    Serial.print(shownSpO2);
    Serial.print(F("%   "));
    Serial.print(medianRatio, 3);
  } else {
    Serial.print(F("--%   -----"));
  }
  Serial.print(F("   "));
  Serial.print(lastPerfusion, 2);
  Serial.print(F("%  "));
  Serial.println(signalStatus());
#endif
}

void printSummary() {
#if !PLOT_SIGNAL
  Serial.println(F("---------- 60 s RESULT ----------"));
  Serial.print(F("HEART RATE (60 s average): "));
  if (average60 > 0) {
    Serial.print(average60);
    Serial.println(F(" BPM"));
  } else {
    Serial.println(F("--"));
  }
  Serial.print(F("Average BPM (0-30 s):  "));
  if (average30 > 0) Serial.println(average30); else Serial.println(F("--"));
  if (bpmSeconds > 0) {
    Serial.print(F("Lowest / highest BPM:  "));
    Serial.print(minBPM);
    Serial.print(F(" / "));
    Serial.println(maxBPM);
  }
  Serial.print(F("Average SpO2:          "));
  if (spo2Seconds > 0) {
    Serial.print((spo2Sum + spo2Seconds / 2) / spo2Seconds);
    Serial.println('%');
    Serial.print(F("Average R:             "));
    Serial.println(ratioSum / spo2Seconds, 3);
  } else {
    Serial.println(F("--"));
  }

  Serial.println(F("-- signal quality --"));
  Serial.print(F("Intervals used:        "));
  Serial.println(intervalCountAll);
  Serial.print(F("Beats accepted:        "));
  Serial.println(beatsAccepted);
  Serial.print(F("Rejected beats / ints: "));
  Serial.print(beatsRejected);
  Serial.print(F(" / "));
  Serial.println(intervalsRejected);
  Serial.print(F("Motion pauses:         "));
  Serial.println(motionEvents);
  unsigned int total = beatsAccepted + beatsRejected;
  unsigned int rejected = beatsRejected + intervalsRejected;
  if (total > 0 && (unsigned long)rejected * 100 > (unsigned long)total * REJECT_WARN_PERCENT) {
    Serial.println(F("WARNING: irregular or noisy signal"));
  }
  Serial.print(F("LED IR / Red:          0x"));
  Serial.print(irPower, HEX);
  Serial.print(F(" / 0x"));
  Serial.println(redPower, HEX);

  Serial.println(F("-- timing check --"));
  Serial.print(F("Lost samples:          "));
  Serial.println(samplesLost);
  Serial.print(F("Sensor ms / real ms:   "));
  Serial.print(sampleClock - sampleClockStart);
  Serial.print(F(" / "));
  Serial.println(millis() - measurementStart);
  Serial.println(F("---------------------------------"));
#endif
}

// =====================================================
// WELCOME SCREEN
// =====================================================

void drawWelcomeScreen() {
  u8g2.firstPage();
  do {
    drawHeartBig(50, 2);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(25, 44, "Heart Monitor");
    u8g2.drawStr(25, 54, "Put finger on");
    u8g2.drawStr(46, 63, "sensor");
    pumpSensor();
  } while (u8g2.nextPage());
}

// =====================================================
// FAKE ECG WAVEFORM
// =====================================================

void updateWaveform() {
  for (int i = 0; i < WAVE_SAMPLES - 1; i++) {
    ecgWave[i] = ecgWave[i + 1];
  }

  int8_t newVal;
  if (spikeStep >= 0 && spikeStep < SPIKE_LEN) {
    newVal = (int8_t)pgm_read_byte(&spikePattern[spikeStep]);
    spikeStep++;
    if (spikeStep >= SPIKE_LEN) spikeStep = -1;
  } else {
    wavePhase += 0.3;
    if (wavePhase > 6.2832) wavePhase -= 6.2832;
    newVal = (int8_t)(1.5 * sin(wavePhase));
  }

  ecgWave[WAVE_SAMPLES - 1] = newVal;
}

// =====================================================
// MEASUREMENT SCREEN
// =====================================================

void drawMeasurementScreen() {
  char bpmBuf[6] = "";
  if (shownBPM > 0) appendInt(bpmBuf, shownBPM);
  else strcpy(bpmBuf, "--");

  char spo2Buf[12] = "SpO2 ";
  if (shownSpO2 > 0) appendInt(spo2Buf, shownSpO2);
  else strcat(spo2Buf, "--");
  strcat(spo2Buf, "%");

  unsigned long seconds = lastSecond;
  if (seconds > 60) seconds = 60;

  char bottomBuf[24] = "";
  if (measurementDone) {
    strcpy(bottomBuf, "DONE AVG:");
    if (average60 > 0) appendInt(bottomBuf, average60);
    else strcat(bottomBuf, "--");
  } else {
    appendInt(bottomBuf, seconds);
    strcat(bottomBuf, "s AVG");
    if (halfDone && average30 > 0) {
      strcat(bottomBuf, "30:");
      appendInt(bottomBuf, average30);
    } else {
      strcat(bottomBuf, ":--");
    }
  }

  const char *status = signalStatus();

  u8g2.firstPage();
  do {
    drawHeartSmall(2, 2, heartFlash);

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(20, 8, status);

    u8g2.setFont(u8g2_font_6x10_tr);
    int spo2W = u8g2.getStrWidth(spo2Buf);
    u8g2.drawStr(126 - spo2W, 10, spo2Buf);

    u8g2.setFont(u8g2_font_logisoso20_tn);
    int textW = u8g2.getStrWidth(bpmBuf);
    int xPos = (128 - textW) / 2;
    if (xPos < 0) xPos = 0;
    u8g2.drawStr(xPos, 36, bpmBuf);

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(xPos + textW + 3, 36, "BPM");

    int baseY = 47;
    for (int i = 0; i < WAVE_SAMPLES - 1; i++) {
      int x1 = i * 2;
      int x2 = (i + 1) * 2;
      int y1 = constrain(baseY + ecgWave[i], 39, 55);
      int y2 = constrain(baseY + ecgWave[i + 1], 39, 55);
      u8g2.drawLine(x1, y1, x2, y2);
    }

    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 63, bottomBuf);

    // Keep reading the sensor while the screen is being sent.
    pumpSensor();
  } while (u8g2.nextPage());
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  Serial.begin(115200);

  u8g2.setBusClock(400000);
  u8g2.begin();
  u8g2.setFontMode(0);

#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);   // never hang forever on a bad I2C wire
#endif

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(20, 34, "Starting...");
  } while (u8g2.nextPage());
  delay(800);

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.drawStr(10, 24, "SENSOR ERROR");
      u8g2.drawStr(10, 40, "Check wiring");
    } while (u8g2.nextPage());

    Serial.println(F("MAX30102 NOT FOUND"));
    while (1) delay(100);
  }

  Serial.println(F("MAX30102 FOUND"));

  // ledMode 2 = Red + IR, pulse width 411 us (18-bit)
  sensor.setup(LED_POWER, SAMPLE_AVERAGE, 2, SENSOR_SAMPLE_RATE, 411, ADC_RANGE);
  sensor.setPulseAmplitudeRed(LED_POWER);
  sensor.setPulseAmplitudeIR(LED_POWER);
  sensor.clearFIFO();

  resetMeasurement();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  pumpSensor();

  unsigned long now = millis();

  if (!fingerPresent) {
#if !PLOT_SIGNAL
    if (now - lastWaitPrint >= 1000) {
      lastWaitPrint = now;
      Serial.print(F("Waiting for finger...  IR="));
      Serial.println(lastIR);
    }
#endif
    if (!welcomeDrawn) {
      drawWelcomeScreen();
      welcomeDrawn = true;
    }
    return;
  }

  if (heartFlash && now - heartFlashStart >= 160) {
    heartFlash = false;
  }

  if (now - lastWaveUpdate >= 25) {
    lastWaveUpdate = now;
    updateWaveform();
  }

  unsigned long elapsed = now - measurementStart;

  if (elapsed >= HALF_WINDOW && !halfDone) {
    halfDone = true;
    average30 = bpmFrom(intervalSumA, intervalCountA);
  }

  // Exactly once per second: refresh the numbers and print one Serial line.
  unsigned long second = elapsed / 1000;
  if (second != lastSecond) {
    lastSecond = second;
    updateEverySecond(second);
  }

  if (elapsed >= MEASURE_WINDOW && !measurementDone) {
    measurementDone = true;
    average60 = bpmFrom(intervalSumAll, intervalCountAll);
    printSummary();
    tone(BUZZER_PIN, 2093, 400);   // long beep: measurement finished
  }

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = now;
    drawMeasurementScreen();
  }
}
