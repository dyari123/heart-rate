# heart-rate

Finger heart-rate (BPM) and SpO2 monitor built on a perfboard.

> SpO2 from a MAX30102 without calibration is an estimate. This is a learning project, not a medical device.

## Parts

- Arduino Nano (ATmega328P)
- MAX30102 pulse oximeter module
- 0.96" SSD1306 128x64 I2C OLED
- Buzzer
- Perfboard + wires

## Wiring

| Part      | Pin  | Nano  |
|-----------|------|-------|
| MAX30102  | VIN  | 5V    |
| MAX30102  | GND  | GND   |
| MAX30102  | SDA  | A4    |
| MAX30102  | SCL  | A5    |
| OLED      | VCC  | 5V    |
| OLED      | GND  | GND   |
| OLED      | SDA  | A4    |
| OLED      | SCL  | A5    |
| Buzzer    | +    | D8    |
| Buzzer    | -    | GND   |

## Libraries (Arduino Library Manager)

- **SparkFun MAX3010x Pulse and Proximity Sensor Library**
- **U8g2** by oliver

Open `heart_monitor/heart_monitor.ino`, board **Arduino Nano**, then upload.
Many Nano clones need **Processor: ATmega328P (Old Bootloader)**.
Serial Monitor: **115200 baud**.

## Using it

1. Power on. The welcome screen says "Put finger on sensor".
2. Rest your fingertip on the sensor. Press lightly and keep still. Pressing hard squeezes the blood out and kills the signal.
3. After a few seconds, BPM and SpO2 appear. Each beat blinks the heart and beeps.
4. At 30 s the screen shows the 0–30 s average. At 60 s it shows the full 60 s average and gives a long beep.
5. If the pulse is lost for 3 s (for example, you moved), BPM goes back to `--` and the sketch relearns your pulse.

BPM and SpO2 on the screen update **exactly once per second**, from 1 s to 60 s.
The Serial Monitor prints one line per second (`time  BPM  SpO2  R  PI  status`) and a summary after 60 s.
**The headline result is the 60 s average**; the live value is an 8-beat average and lags a few seconds by design.

How the reading is kept accurate:

- **No lost samples.** The sketch reads the MAX30102's own 32-sample FIFO directly. The library's buffer holds only 4 samples on a Nano.
- **LED auto-tune.** While the signal settles, IR and Red currents are tuned separately to a raw level of 100,000–200,000.
- **Filter.** A 2nd-order Butterworth low-pass filter at 5 Hz removes noise.
- **Beat timing.** Each beat is timed at the steepest point of the pulse upstroke, to about 1 ms.
- **Quality checks.** Beats are skipped when:
  - the pulse is too weak (perfusion index below 0.2%, shown as `WEAK`);
  - the top is clipped (`CLIPPED`);
  - the finger moved (baseline jump over 2% in 0.5 s pauses detection for 1 s, shown as `MOTION`).
- **Rejection count.** Rejected beats and intervals are counted in the summary. Above 15% it prints `WARNING: irregular or noisy signal`.

SpO2 appears after 5 good beats. It is the median of the last 8 beats.
For a healthy person at rest, **95–100%** is normal, and 96–99% is the most common reading.

## Timing check (do this first)

The summary ends with:

```
Lost samples:          0
Sensor ms / real ms:   60012 / 60000
```

- `Lost samples` must be 0.
- If sensor ms and real ms differ by more than 0.5%, run 3–4 times, average `sensor ms / real ms`, and put it in `CLOCK_FIX`.
  The Nano's own clock is only about ±0.5% accurate, so ignore smaller differences.

## Hardware tips

- Hold the finger still with a clip or foam cradle, at light, constant pressure.
- Block ambient light with black tape or a dark cap.
- Warm cold fingers first.
- Keep I2C wires short and twisted with GND.
- Add a 100 µF capacitor across 5V/GND near the sensor.
- Ideally, drive the buzzer through a transistor.

## Calibrating SpO2

This sensor has no factory calibration, so each module can read a little high or low.
To calibrate it against a real fingertip pulse oximeter (the pharmacy kind):

1. Wear the real oximeter on one hand and this sensor on the other. Sit still for the full 60 s.
2. Note the oximeter's SpO2 and `Average R` from the summary.
3. Compute `A = oximeterSpO2 + 25 * R`.
   Example: 97% and R = 0.40 gives `A = 97 + 25 * 0.40 = 107`.
4. Repeat on 10+ sessions on different days, average A, and put it in `SPO2_A`.

This fixes the offset only. Healthy people are all near 96–99%, so the slope can't be calibrated at home.
**Never hold your breath to lower your SpO2 for this.**
Redo the calibration after changing the module, filter or LED logic.
Expect about 2–4% accuracy near normal values.

## Testing against a reference

Run the device and a reference (chest strap, fingertip oximeter, smartwatch, or a manual 60 s pulse count) together for 60 s.
Do 10 runs at rest and 5 after light exercise, then log the results:

| Run | Person | Condition | Your BPM | Reference BPM | Difference | Your SpO2 | Reference SpO2 |
|-----|--------|-----------|----------|---------------|------------|-----------|----------------|
| 1   |        | rest      |          |               |            |           |                |

A good result is within ±1 BPM at rest and ±2 BPM after exercise.

## Tuning

- **Finger not detected, or detected with no finger:** watch `IR=` on the Serial Monitor, then change `FINGER_ON_LEVEL` / `FINGER_OFF_LEVEL`.
- **To see the pulse wave:** set `#define PLOT_SIGNAL 1` and open the Serial Plotter. You should see a clean bump for each beat.
- **`WEAK` all the time:** warm the finger, block ambient light, press more lightly. `MIN_PERFUSION` is the limit (0.2%).
- **IR near 262,143 even at low LED current:** change `ADC_RANGE` to 8192 or 16384.
