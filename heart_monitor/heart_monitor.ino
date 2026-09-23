/*
  Heart Monitor - Arduino Nano + MAX30102 + 0.96" SSD1306 OLED + buzzer

  Wiring (everything on the same I2C bus):
    MAX30102  VIN -> 5V (or 3.3V, see README), GND -> GND, SDA -> A4, SCL -> A5
    OLED      VCC -> 5V,  GND -> GND, SDA -> A4, SCL -> A5
    Buzzer    +   -> D8,  -   -> GND

  Libraries (Arduino Library Manager):
    "SparkFun MAX3010x Pulse and Proximity Sensor Library"
    "U8g2" by oliver

  Serial Monitor: 115200 baud.

  SpO2 is an estimate from an uncalibrated hobby sensor. It is NOT a medical device.
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
const byte LED_POWER = 0x1F;          // ~6.4 mA, same for Red and IR (keeps SpO2 math simple)
const byte SAMPLE_AVERAGE = 4;
const int SENSOR_SAMPLE_RATE = 400;
const unsigned int SAMPLE_MS = 10;    // 1000 / (400 / 4)

// Finger detection. Check "IR=" on the Serial Monitor with and without a
// finger and adjust if your module reads very differently.
const long FINGER_ON_LEVEL = 20000;
const long FINGER_OFF_LEVEL = 12000;
const byte FINGER_ON_SAMPLES = 10;    // 0.10 s above ON level before we start
const byte FINGER_OFF_SAMPLES = 25;   // 0.25 s below OFF level before we give up

// Beat detection
const unsigned int SETTLE_MS = 1500;          // ignore the first moments while the finger settles
const float DC_ALPHA = 0.02;                  // slow baseline (removes drift / finger pressure)
const float LP_ALPHA = 0.25;                  // smoothing (removes noise)
const float MIN_HYSTERESIS = 40;              // smallest swing that counts as a turn in the wave
const float MIN_PULSE_AMPLITUDE = 80;         // smallest valley-to-peak that counts as a beat
const unsigned int MIN_BEAT_INTERVAL = 300;   // 200 BPM
const unsigned int MAX_BEAT_INTERVAL = 1500;  // 40 BPM
const unsigned int SIGNAL_LOST_MS = 3000;     // no beat for this long -> show "--" and re-learn
const float INTERVAL_TOLERANCE = 0.25;        // beat interval may differ 25% from the median

// SpO2 = SPO2_A - SPO2_B * R   (R = ratio of Red to IR pulse strength)
// 110 - 25*R is the standard textbook approximation. To calibrate against a
// real fingertip oximeter, see the README ("Calibrating SpO2").
const float SPO2_A = 110.0;
const float SPO2_B = 25.0;
const byte SPO2_MIN_BEATS = 5;                // good beats needed before SpO2 is shown

// Measurement
const unsigned long HALF_WINDOW = 30000;
const unsigned long MEASURE_WINDOW = 60000;
const unsigned long DISPLAY_INTERVAL = 100;
const unsigned long SERIAL_INTERVAL = 500;

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
unsigned long intervalSumA = 0;   // beats in 0-30 s
unsigned int beatCountA = 0;
unsigned long intervalSumAll = 0; // beats in 0-60 s
unsigned int beatCountAll = 0;

int currentBPM = 0;
int currentSpO2 = 0;

// Time base built from the sensor's own sample rate, so beat timing stays
// exact even while the display is busy.
unsigned long sampleClock = 0;
uint32_t lastIR = 0;
uint32_t lastRed = 0;

// Filters
bool filtersReady = false;
float irDC = 0, redDC = 0;   // baselines
float irAC = 0, redAC = 0;   // pulse signal (baseline removed, smoothed, flipped so beats point up)
unsigned long settleUntil = 0;

// Peak / valley detector (with hysteresis)
bool lookingForPeak = true;
bool haveValley = false;
float extremeIR = 0;
unsigned long extremeTime = 0;
float valleyIR = 0;
float extremeRed = 0, valleyRed = 0;   // Red taken at the same moments as the IR peak / valley
float pulseAmplitude = 0;    // learned typical beat size (0 = still learning)
unsigned long lastBeatTime = 0;
unsigned long lastActivity = 0;

// Last beat-to-beat intervals (ms)
#define IBI_SIZE 8
#define IBI_DISPLAY 4        // BPM on screen = average of the last 4 intervals
unsigned int ibi[IBI_SIZE];
byte ibiCount = 0;
byte ibiHead = 0;
byte outlierStreak = 0;

// SpO2 ratio of the last beats (median is used, so one bad beat can't move it)
#define RATIO_SIZE 8
float ratios[RATIO_SIZE];
byte ratioCount = 0;
byte ratioHead = 0;
float lastRatio = 0;

// Screen effects
bool heartFlash = false;
unsigned long heartFlashStart = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastWaveUpdate = 0;

// Fake ECG waveform
#define WAVE_SAMPLES 64
int8_t ecgWave[WAVE_SAMPLES];
int spikeStep = -1;
float wavePhase = 0;

const int8_t spikePattern[] PROGMEM = {0, -2, 0, 2, -8, 4, -2, 0};
const int SPIKE_LEN = sizeof(spikePattern);

// =====================================================
// RESET / START
// =====================================================

void clearRatios() {
  ratioCount = 0;
  ratioHead = 0;
  lastRatio = 0;
}

void clearIntervals() {
  ibiCount = 0;
  ibiHead = 0;
  outlierStreak = 0;
}

void resetDetector() {
  filtersReady = false;
  irDC = redDC = 0;
  irAC = redAC = 0;

  lookingForPeak = true;
  haveValley = false;
  extremeIR = 0;
  extremeTime = 0;
  valleyIR = 0;
  extremeRed = valleyRed = 0;
  pulseAmplitude = 0;
  lastBeatTime = 0;
  lastActivity = sampleClock;

  clearIntervals();
  clearRatios();
  currentBPM = 0;
  currentSpO2 = 0;
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
  beatCountA = 0;
  intervalSumAll = 0;
  beatCountAll = 0;

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
  Serial.println(F("FINGER DETECTED - 60s MEASUREMENT STARTED"));
}

// Heart rhythm lost (moved finger, pressed too hard...). Show "--" instead of
// freezing on an old number, and re-learn the pulse size from scratch.
void signalLost() {
  if (currentBPM > 0) Serial.println(F("NO PULSE - hold still"));
  currentBPM = 0;
  currentSpO2 = 0;
  clearRatios();
  pulseAmplitude = 0;
  lastBeatTime = 0;
  lastActivity = sampleClock;
  clearIntervals();
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

  lastRatio = ratio;
  ratios[ratioHead] = ratio;
  ratioHead = (ratioHead + 1) % RATIO_SIZE;
  if (ratioCount < RATIO_SIZE) ratioCount++;

  if (ratioCount >= SPO2_MIN_BEATS) {
    float sorted[RATIO_SIZE];
    for (byte i = 0; i < ratioCount; i++) sorted[i] = ratios[i];
    for (byte i = 1; i < ratioCount; i++) {
      float v = sorted[i];
      byte j = i;
      while (j > 0 && sorted[j - 1] > v) {
        sorted[j] = sorted[j - 1];
        j--;
      }
      sorted[j] = v;
    }
    float r = sorted[ratioCount / 2];

    float spo2f = SPO2_A - SPO2_B * r;
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
  for (byte i = 1; i < ibiCount; i++) {
    unsigned int v = sorted[i];
    byte j = i;
    while (j > 0 && sorted[j - 1] > v) {
      sorted[j] = sorted[j - 1];
      j--;
    }
    sorted[j] = v;
  }
  return sorted[ibiCount / 2];
}

void addInterval(unsigned int interval) {
  // Reject a single odd interval (missed or extra beat), but if the rhythm
  // really changed, 3 odd ones in a row start a fresh history.
  if (ibiCount >= 3) {
    unsigned int med = medianInterval();
    unsigned int diff = (interval > med) ? interval - med : med - interval;
    if (diff > med * INTERVAL_TOLERANCE) {
      outlierStreak++;
      if (outlierStreak < 3) return;
      clearIntervals();
    }
  }
  outlierStreak = 0;

  ibi[ibiHead] = interval;
  ibiHead = (ibiHead + 1) % IBI_SIZE;
  if (ibiCount < IBI_SIZE) ibiCount++;

  // Shown from the very first interval, then averaged over the last 4.
  byte n = (ibiCount < IBI_DISPLAY) ? ibiCount : IBI_DISPLAY;
  unsigned long sum = 0;
  for (byte i = 1; i <= n; i++) {
    sum += ibi[(ibiHead + IBI_SIZE - i) % IBI_SIZE];
  }
  currentBPM = (int)((60000UL * n + sum / 2) / sum);

  // Official 30 s / 60 s averages only use beats that passed every check.
  if (!measurementDone) {
    unsigned long elapsed = millis() - measurementStart;
    if (elapsed < HALF_WINDOW) {
      intervalSumA += interval;
      beatCountA++;
    }
    if (elapsed < MEASURE_WINDOW) {
      intervalSumAll += interval;
      beatCountAll++;
    }
  }

  Serial.print(F("HEARTBEAT  BPM="));
  Serial.print(currentBPM);
  Serial.print(F("  SpO2="));
  Serial.print(currentSpO2);
  Serial.print(F("  R="));
  Serial.println(lastRatio, 3);
}

// A full valley -> peak pulse was found in the IR signal.
void onPulse(float amplitude, float redAmplitude, unsigned long peakTime) {
  if (amplitude < MIN_PULSE_AMPLITUDE) return;

  // Second bump of the same heartbeat (dicrotic notch) or noise.
  if (lastBeatTime != 0 && peakTime - lastBeatTime < MIN_BEAT_INTERVAL) return;

  // Much bigger or smaller than the usual beat = finger movement.
  if (pulseAmplitude > 0 &&
      (amplitude > pulseAmplitude * 3 || amplitude < pulseAmplitude * 0.3)) return;

  pulseAmplitude = (pulseAmplitude == 0) ? amplitude : pulseAmplitude * 0.8 + amplitude * 0.2;
  lastActivity = sampleClock;

  beatFeedback();
  updateSpO2(amplitude, redAmplitude);

  if (lastBeatTime == 0) {
    lastBeatTime = peakTime;
    return;
  }

  unsigned long interval = peakTime - lastBeatTime;
  lastBeatTime = peakTime;

  // A gap of about 2 or 3 normal beats means weak beats in between were not
  // detected. Split it into the missed beats instead of throwing it away
  // (or worse, counting it as one very slow beat) so BPM keeps updating.
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

  if (interval <= MAX_BEAT_INTERVAL) addInterval((unsigned int)interval);
}

// =====================================================
// SIGNAL PROCESSING (called for EVERY sensor sample)
// =====================================================

void processSample(uint32_t ir, uint32_t red) {
  if (!filtersReady) {
    irDC = ir;
    redDC = red;
    irAC = 0;
    redAC = 0;
    filtersReady = true;
    settleUntil = sampleClock + SETTLE_MS;
    lastActivity = settleUntil;
    return;
  }

  irDC += DC_ALPHA * ((float)ir - irDC);
  redDC += DC_ALPHA * ((float)red - redDC);

  // Blood absorbs light, so every heartbeat makes the reading DROP.
  // Flip the sign so a heartbeat is an upward peak.
  irAC += LP_ALPHA * ((irDC - (float)ir) - irAC);
  redAC += LP_ALPHA * ((redDC - (float)red) - redAC);

#if PLOT_SIGNAL
  Serial.println(irAC);
#endif

  if ((long)(sampleClock - settleUntil) < 0) return;

  if ((long)(sampleClock - lastActivity) > (long)SIGNAL_LOST_MS) signalLost();

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
      extremeTime = sampleClock;
    } else if (extremeIR - irAC > hysteresis) {
      float amplitude = extremeIR - valleyIR;
      float redAmplitude = extremeRed - valleyRed;
      unsigned long peakTime = extremeTime;

      lookingForPeak = false;
      extremeIR = irAC;
      extremeRed = redAC;

      if (haveValley) onPulse(amplitude, redAmplitude, peakTime);
    }
  } else {
    if (irAC <= extremeIR) {
      extremeIR = irAC;
      extremeRed = redAC;
    } else if (irAC - extremeIR > hysteresis) {
      valleyIR = extremeIR;
      valleyRed = extremeRed;
      haveValley = true;

      lookingForPeak = true;
      extremeIR = irAC;
      extremeRed = redAC;
      extremeTime = sampleClock;
    }
  }
}

void handleSample(uint32_t ir, uint32_t red) {
  sampleClock += SAMPLE_MS;
  lastIR = ir;
  lastRed = red;

  if (!fingerPresent) {
    if ((long)ir > FINGER_ON_LEVEL) {
      if (++fingerOnCount >= FINGER_ON_SAMPLES) startMeasurement();
    } else {
      fingerOnCount = 0;
    }
    return;
  }

  if ((long)ir < FINGER_OFF_LEVEL) {
    if (++fingerOffCount >= FINGER_OFF_SAMPLES) {
      Serial.println(F("FINGER REMOVED"));
      resetMeasurement();
    }
    return;
  }
  fingerOffCount = 0;

  processSample(ir, red);
}

// Read EVERY new sample waiting in the sensor. The library only keeps 4 on a
// Nano, so this is also called between the display's page transfers.
void pumpSensor() {
  sensor.check();
  while (sensor.available()) {
    uint32_t ir = sensor.getFIFOIR();
    uint32_t red = sensor.getFIFORed();
    sensor.nextSample();
    handleSample(ir, red);
  }
}

// =====================================================
// WELCOME SCREEN
// =====================================================

void drawWelcomeScreen() {
  u8g2.firstPage();
  do {
    drawHeartBig(50, 2);
    u8g2.setFont(u8g2_font_6x10_tf);
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
  char bpmBuf[5];
  if (currentBPM > 0) {
    itoa(currentBPM, bpmBuf, 10);
  } else {
    strcpy(bpmBuf, "--");
  }

  char spo2Buf[12];
  if (currentSpO2 > 0) {
    sprintf(spo2Buf, "SpO2 %d%%", currentSpO2);
  } else {
    strcpy(spo2Buf, "SpO2 --%");
  }

  unsigned long seconds = 0;
  if (measurementStart != 0) {
    seconds = (millis() - measurementStart) / 1000;
    if (seconds > 60) seconds = 60;
  }

  char bottomBuf[24];
  if (measurementDone) {
    if (average60 > 0) sprintf(bottomBuf, "DONE 60s AVG:%d", average60);
    else strcpy(bottomBuf, "DONE 60s AVG:--");
  } else if (halfDone && average30 > 0) {
    sprintf(bottomBuf, "%lus AVG30:%d", seconds, average30);
  } else {
    sprintf(bottomBuf, "%lus AVG:--", seconds);
  }

  u8g2.firstPage();
  do {
    drawHeartSmall(2, 2, heartFlash);

    u8g2.setFont(u8g2_font_6x10_tf);
    int spo2W = u8g2.getStrWidth(spo2Buf);
    u8g2.drawStr(126 - spo2W, 10, spo2Buf);

    u8g2.setFont(u8g2_font_logisoso20_tr);
    int textW = u8g2.getStrWidth(bpmBuf);
    int xPos = (128 - textW) / 2;
    if (xPos < 0) xPos = 0;
    u8g2.drawStr(xPos, 36, bpmBuf);

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(xPos + textW + 3, 36, "BPM");

    int baseY = 47;
    for (int i = 0; i < WAVE_SAMPLES - 1; i++) {
      int x1 = i * 2;
      int x2 = (i + 1) * 2;
      int y1 = constrain(baseY + ecgWave[i], 39, 55);
      int y2 = constrain(baseY + ecgWave[i + 1], 39, 55);
      u8g2.drawLine(x1, y1, x2, y2);
    }

    u8g2.setFont(u8g2_font_6x10_tf);
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
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(20, 34, "Starting...");
  } while (u8g2.nextPage());
  delay(800);

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(10, 24, "SENSOR ERROR");
      u8g2.drawStr(10, 40, "Check wiring");
    } while (u8g2.nextPage());

    Serial.println(F("MAX30102 NOT FOUND"));
    while (1) delay(100);
  }

  Serial.println(F("MAX30102 FOUND"));

  // ledMode 2 = Red + IR, pulse width 411 us (18-bit), ADC range 4096 nA
  sensor.setup(LED_POWER, SAMPLE_AVERAGE, 2, SENSOR_SAMPLE_RATE, 411, 4096);
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

#if !PLOT_SIGNAL
  if (now - lastSerialPrint >= SERIAL_INTERVAL) {
    lastSerialPrint = now;
    Serial.print(F("IR="));
    Serial.print(lastIR);
    Serial.print(F("  RED="));
    Serial.println(lastRed);
  }
#endif

  if (!fingerPresent) {
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
    average30 = (intervalSumA > 0) ? (int)((60000.0 * beatCountA) / intervalSumA + 0.5) : 0;
    Serial.print(F("AVG (0-30s) = "));
    Serial.println(average30);
  }

  if (elapsed >= MEASURE_WINDOW && !measurementDone) {
    measurementDone = true;
    average60 = (intervalSumAll > 0) ? (int)((60000.0 * beatCountAll) / intervalSumAll + 0.5) : 0;
    Serial.print(F("AVG (0-60s) = "));
    Serial.println(average60);
    tone(BUZZER_PIN, 2093, 400);   // long beep: measurement finished
  }

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = now;
    drawMeasurementScreen();
  }
}
