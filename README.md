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

BPM updates **on every heartbeat**, so about once per second at 60 BPM, faster when the heart is faster.
The number is the average of the last 4 beat-to-beat intervals.

## Tuning

- **Finger not detected, or detected with no finger:** watch `IR=` on the Serial Monitor, then change `FINGER_ON_LEVEL` / `FINGER_OFF_LEVEL`.
- **To see the pulse wave:** set `#define PLOT_SIGNAL 1` and open the Serial Plotter. You should see a clean bump for each beat.
- **Pulse too weak (small bumps):** raise `LED_POWER` (for example `0x3F`).
