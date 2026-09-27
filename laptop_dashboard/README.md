# Laptop dashboard

Shows the heart monitor on your laptop screen with big coloured numbers and live graphs.
It reads the lines the Arduino already prints on USB, so **the Arduino sketch does not change**.

```
Arduino  --USB cable-->  laptop  -->  heart_dashboard.py (Tkinter window)
```

## Steps

1. Install Python 3 from https://www.python.org/downloads/ (on Windows, tick **"Add python.exe to PATH"**).
2. Open a terminal (Windows: press Start, type `cmd`, Enter) and install the serial library:
   ```
   pip install pyserial
   ```
3. Plug the Arduino into the laptop with the USB cable (the sketch is already uploaded).
4. **Close the Arduino IDE Serial Monitor and Serial Plotter.** Only one program can use the port.
5. Go to this folder and run:
   ```
   python heart_dashboard.py
   ```
6. Pick the Arduino's port (Windows: `COM3`, `COM5`...; Mac: `/dev/cu.usbserial...`; Linux: `/dev/ttyUSB0`) and press **Connect**.

Press **Demo** to try the window without the Arduino. **Record CSV** saves every second to a file that opens in Excel.

## Problems

| What you see | Fix |
|---|---|
| `Port busy` / `Access denied` | Close the Arduino Serial Monitor, press Disconnect, then Connect again |
| No port in the list | Replug the cable, press ↻. Clone Nanos need the CH340 driver |
| Connected but nothing shows | Wait 2 s (the Nano restarts when the port opens). Check `Serial.begin(115200)` |
| `pip` not found | Use `py -m pip install pyserial` (Windows) or `python3 -m pip install pyserial` (Mac/Linux) |
| `No module named tkinter` (Linux) | `sudo apt install python3-tk` |

Hobby project, not a medical device.
