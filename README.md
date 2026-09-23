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
The Serial Monitor prints one line per second (`time  BPM  SpO2  R`) and a summary after 60 s.

- Each beat is timed at the steepest point of the pulse upstroke, found to about 1 ms precision.
- BPM is the trimmed mean of the last 8 good beat-to-beat intervals (the shortest and longest are dropped).
- If a weak beat is not detected, the gap is split into the missed beats, so BPM keeps updating.
- The first BPM appears about 3–4 s after the finger goes down (the signal must settle and 2 beats are needed).

SpO2 appears after 5 good beats. It is the median of the last 8 beats.
For a healthy person at rest, **95–100%** is normal, and 96–99% is the most common reading.

## Calibrating SpO2

This sensor has no factory calibration, so each module can read a little high or low.
To calibrate it against a real fingertip pulse oximeter (the pharmacy kind):

1. Open the Serial Monitor. Every second prints a line like `12s     72   97%   0.512` (the last number is R).
2. Measure one finger with the real oximeter and another finger with this device, at the same time.
3. After about 30 s, note the typical `R` value and the real oximeter's SpO2.
4. In the sketch, set `SPO2_A = realSpO2 + 25 * R`.
   Example: the real oximeter shows 97 and R is about 0.40, so `SPO2_A = 97 + 25 * 0.40 = 107`.

## Tuning

- **Finger not detected, or detected with no finger:** watch `IR=` on the Serial Monitor, then change `FINGER_ON_LEVEL` / `FINGER_OFF_LEVEL`.
- **To see the pulse wave:** set `#define PLOT_SIGNAL 1` and open the Serial Plotter. You should see a clean bump for each beat.
- **Pulse too weak (small bumps):** raise `LED_POWER` (for example `0x3F`).
