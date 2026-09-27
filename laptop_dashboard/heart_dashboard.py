"""
Health Monitor Dashboard - shows the Arduino heart monitor on the laptop screen.

It reads the lines the Arduino already prints on the Serial port, for example
    12s BPM=72 SpO2=98 Rx1000=487 PI=85 Temp=365 ok
    Waiting... IR=3000 Temp=251
    --- 30s AVG ---
and shows them as big, colourful tiles with live graphs.

Run:        python heart_dashboard.py
Needs:      pip install pyserial        (Tkinter comes with Python)
Important:  close the Arduino IDE Serial Monitor first - only one program
            can use the USB port at a time.

This is a hobby project, NOT a medical device.
"""

import csv
import os
import math
import queue
import random
import re
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # the app still opens (demo mode works) and explains what to install
    serial = None

BAUD_RATE = 115200          # must match Serial.begin(...) in the Arduino sketch
GRAPH_SECONDS = 60          # how much history the graphs show

# Colours
BG = "#101418"
PANEL = "#1b2229"
TEXT = "#e8edf2"
MUTED = "#8a96a3"
RED = "#ff5c6c"
BLUE = "#4da3ff"
ORANGE = "#ffa94d"
PURPLE = "#b18cff"
GREEN = "#3ddc84"
YELLOW = "#ffd43b"

# Lines the Arduino prints (see updateEverySecond() / loop() in the sketch)
LIVE_RE = re.compile(
    r"(\d+)s BPM=(\d+) SpO2=(\d+) Rx1000=(-?\d+) PI=(-?\d+) Temp=(-?\d+) (\w+)")
WAIT_RE = re.compile(r"Waiting\.\.\. IR=(\d+) Temp=(-?\d+)")
AVG_BPM_RE = re.compile(r"Avg BPM:\s*(\d+)")
AVG_SPO2_RE = re.compile(r"Avg SpO2:\s*(\d+|--)")
USED_RE = re.compile(r"Used/Rejected:\s*(\d+)/(\d+)")

STATUS_TEXT = {
    "ok": ("Signal OK", GREEN),
    "weak": ("Weak pulse - warm finger, block light", YELLOW),
    "move": ("Hold still", YELLOW),
    "clip": ("Press lighter", YELLOW),
}


def temp_text(tenths):
    """Temperature arrives as tenths of a degree (365 = 36.5 C). -9999 = not read."""
    if tenths < -1000:
        return "--"
    return f"{tenths / 10:.1f}"


class SerialReader(threading.Thread):
    """Reads lines from the USB port in the background and puts them in a queue."""

    def __init__(self, port, lines):
        super().__init__(daemon=True)
        self.port = port
        self.lines = lines
        self.running = True

    def run(self):
        try:
            with serial.Serial(self.port, BAUD_RATE, timeout=1) as ser:
                self.lines.put(("info", f"Connected: {os.path.basename(self.port)}"))
                while self.running:
                    raw = ser.readline()
                    if raw:
                        self.lines.put(("line", raw.decode("utf-8", errors="replace").strip()))
        except Exception as exc:  # port busy, cable pulled, ...
            self.lines.put(("error", str(exc)))

    def stop(self):
        self.running = False


class DemoReader(threading.Thread):
    """Pretends to be the Arduino, so the dashboard can be tried without hardware."""

    def __init__(self, lines):
        super().__init__(daemon=True)
        self.lines = lines
        self.running = True

    def run(self):
        put = lambda text: self.lines.put(("line", text))
        put("MAX30102 FOUND")
        put("MLX90614 FOUND")
        for _ in range(3):
            if not self.running:
                return
            put(f"Waiting... IR=3000 Temp={250 + random.randint(-2, 2)}")
            time.sleep(1)
        put("")
        put("FINGER DETECTED")
        sec, used = 0, 0
        while self.running:
            sec += 1
            bpm = 0 if sec < 4 else int(72 + 4 * math.sin(sec / 6) + random.randint(-1, 1))
            spo2 = 0 if sec < 6 else random.choice([97, 98, 98, 99])
            temp = 330 + min(sec, 10) * 3 + random.randint(-1, 1)
            status = "move" if sec % 23 == 0 else "ok"
            put(f"{sec % 30}s BPM={bpm} SpO2={spo2} Rx1000={random.randint(470, 510)} "
                f"PI={random.randint(70, 95)} Temp={temp} {status}")
            used += 1
            if sec % 30 == 0:
                put("--- 30s AVG ---")
                put(f"Avg BPM: {72 + random.randint(-1, 1)}")
                put(f"Used/Rejected: {used}/{random.randint(0, 2)}")
                put("Avg SpO2: 98")
                used = 0
            time.sleep(1)

    def stop(self):
        self.running = False


class Tile(tk.Frame):
    """One big coloured number with a title and a unit."""

    def __init__(self, parent, title, unit, colour):
        super().__init__(parent, bg=PANEL, padx=18, pady=12)
        self.colour = colour
        tk.Label(self, text=title, bg=PANEL, fg=MUTED, font=("Segoe UI", 13)).pack(anchor="w")
        row = tk.Frame(self, bg=PANEL)
        row.pack(anchor="w")
        self.value = tk.Label(row, text="--", bg=PANEL, fg=colour, font=("Segoe UI", 54, "bold"))
        self.value.pack(side="left")
        tk.Label(row, text=unit, bg=PANEL, fg=MUTED, font=("Segoe UI", 16)).pack(
            side="left", anchor="s", pady=(0, 14), padx=(6, 0))
        self.note = tk.Label(self, text="", bg=PANEL, fg=MUTED, font=("Segoe UI", 11))
        self.note.pack(anchor="w")

    def set(self, text, note="", colour=None):
        self.value.config(text=text, fg=colour or self.colour)
        self.note.config(text=note)


class Graph(tk.Canvas):
    """Simple scrolling line graph of the last GRAPH_SECONDS values."""

    def __init__(self, parent, title, colour, low, high, unit):
        super().__init__(parent, bg=PANEL, height=170, highlightthickness=0)
        self.title, self.colour, self.low, self.high, self.unit = title, colour, low, high, unit
        self.values = []
        self.bind("<Configure>", lambda e: self.redraw())

    def add(self, value):
        self.values.append(value)
        self.values = self.values[-GRAPH_SECONDS:]
        self.redraw()

    def clear(self):
        self.values = []
        self.redraw()

    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        left, right, top, bottom = 44, 12, 28, 22
        self.create_text(12, 14, text=self.title, fill=MUTED, anchor="w", font=("Segoe UI", 12))

        # Auto-range around the data, but never smaller than the default range
        real = [v for v in self.values if v is not None]
        low, high = self.low, self.high
        if real:
            low, high = min(low, min(real) - 2), max(high, max(real) + 2)

        def y_of(v):
            return top + (high - v) / (high - low) * (h - top - bottom)

        for i in range(5):  # grid lines with labels
            v = low + (high - low) * i / 4
            y = y_of(v)
            self.create_line(left, y, w - right, y, fill="#2a333c")
            self.create_text(left - 6, y, text=f"{v:.0f}", fill=MUTED, anchor="e", font=("Segoe UI", 9))
        self.create_text(w - right, h - 8, text=f"last {GRAPH_SECONDS} s", fill=MUTED,
                         anchor="e", font=("Segoe UI", 9))

        step = (w - left - right) / max(GRAPH_SECONDS - 1, 1)
        points = []
        for i, v in enumerate(self.values):
            x = left + (GRAPH_SECONDS - len(self.values) + i) * step
            if v is None:  # gap: no reading that second
                if len(points) >= 4:
                    self.create_line(points, fill=self.colour, width=3, smooth=True)
                points = []
                continue
            points += [x, y_of(v)]
        if len(points) >= 4:
            self.create_line(points, fill=self.colour, width=3, smooth=True)
        if points:
            x, y = points[-2], points[-1]
            self.create_oval(x - 5, y - 5, x + 5, y + 5, fill=self.colour, outline="")


class Dashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Health Monitor Dashboard")
        self.configure(bg=BG)
        self.geometry("1100x760")
        self.minsize(900, 640)

        self.lines = queue.Queue()
        self.reader = None
        self.csv_file = None
        self.csv_writer = None
        self.in_avg_block = False
        self.heart_on = False

        self.build_top_bar()
        self.build_tiles()
        self.build_graphs()
        self.build_log()
        self.refresh_ports()
        self.after(100, self.poll)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- layout ----------

    def build_top_bar(self):
        bar = tk.Frame(self, bg=BG, padx=16, pady=10)
        bar.pack(fill="x")
        self.heart = tk.Label(bar, text="♥", bg=BG, fg="#5a2a30", font=("Segoe UI", 28))
        self.heart.pack(side="left")
        tk.Label(bar, text="Health Monitor", bg=BG, fg=TEXT,
                 font=("Segoe UI", 20, "bold")).pack(side="left", padx=(6, 24))

        tk.Label(bar, text="Port:", bg=BG, fg=MUTED, font=("Segoe UI", 11)).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=24, state="readonly")
        self.port_box.pack(side="left", padx=6)
        ttk.Button(bar, text="↻", width=3, command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=6)
        ttk.Button(bar, text="Demo", command=self.start_demo).pack(side="left")
        self.record_btn = ttk.Button(bar, text="Record CSV", command=self.toggle_record)
        self.record_btn.pack(side="left", padx=6)

    def build_tiles(self):
        grid = tk.Frame(self, bg=BG, padx=12)
        grid.pack(fill="x")
        self.bpm_tile = Tile(grid, "Heart rate", "BPM", RED)
        self.spo2_tile = Tile(grid, "Blood oxygen (SpO2)", "%", BLUE)
        self.temp_tile = Tile(grid, "Temperature", "°C", ORANGE)
        self.avg_tile = Tile(grid, "30 s average", "BPM", PURPLE)
        for col, tile in enumerate((self.bpm_tile, self.spo2_tile, self.temp_tile, self.avg_tile)):
            tile.grid(row=0, column=col, sticky="nsew", padx=4, pady=4)
            grid.columnconfigure(col, weight=1, uniform="tiles")

        info = tk.Frame(self, bg=BG, padx=16, pady=4)
        info.pack(fill="x")
        self.status_label = tk.Label(info, text="Waiting for data...", bg=BG, fg=MUTED,
                                     font=("Segoe UI", 15, "bold"))
        self.status_label.pack(side="left")
        self.state_label = tk.Label(info, text="Not connected", bg=BG, fg=MUTED, font=("Segoe UI", 11))
        self.state_label.pack(side="right")
        self.clock_label = tk.Label(info, text="", bg=BG, fg=MUTED, font=("Segoe UI", 13))
        self.clock_label.pack(side="right", padx=(0, 24))

    def build_graphs(self):
        area = tk.Frame(self, bg=BG, padx=12)
        area.pack(fill="both", expand=True)
        self.bpm_graph = Graph(area, "Heart rate (BPM)", RED, 50, 110, "BPM")
        self.temp_graph = Graph(area, "Temperature (°C)", ORANGE, 20, 40, "C")
        self.bpm_graph.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.temp_graph.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        area.columnconfigure(0, weight=3)
        area.columnconfigure(1, weight=2)
        area.rowconfigure(0, weight=1)

    def build_log(self):
        box = tk.Frame(self, bg=BG, padx=16, pady=8)
        box.pack(fill="x")
        tk.Label(box, text="Messages from the Arduino", bg=BG, fg=MUTED,
                 font=("Segoe UI", 11)).pack(anchor="w")
        self.log = tk.Text(box, height=7, bg=PANEL, fg=TEXT, relief="flat",
                           font=("Consolas", 10), state="disabled")
        self.log.pack(fill="x")
        tk.Label(self, text="Hobby project - not a medical device.", bg=BG, fg=MUTED,
                 font=("Segoe UI", 9)).pack(pady=(0, 6))

    # ---------- connection ----------

    def refresh_ports(self):
        if serial is None:
            self.port_box["values"] = []
            self.state_label.config(text="pyserial missing: run  pip install pyserial", fg=YELLOW)
            return
        ports = [f"{p.device} - {p.description}" for p in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            # Prefer something that looks like an Arduino / USB-serial chip
            likely = [p for p in ports if any(k in p for k in ("CH340", "Arduino", "USB", "ACM", "usbserial"))]
            self.port_var.set((likely or ports)[0])

    def toggle_connect(self):
        if self.reader:
            self.stop_reader()
            return
        if serial is None:
            messagebox.showerror("pyserial missing", "Open a terminal and run:\n\npip install pyserial")
            return
        choice = self.port_var.get()
        if not choice:
            messagebox.showinfo("No port", "Plug in the Arduino, then press ↻ and pick its port.")
            return
        port = choice.split(" - ")[0]
        self.reset_view()
        self.reader = SerialReader(port, self.lines)
        self.reader.start()
        self.connect_btn.config(text="Disconnect")
        self.state_label.config(text=f"Connecting: {os.path.basename(port)}...", fg=YELLOW)

    def start_demo(self):
        self.stop_reader()
        self.reset_view()
        self.reader = DemoReader(self.lines)
        self.reader.start()
        self.connect_btn.config(text="Stop demo")
        self.state_label.config(text="DEMO (fake data)", fg=YELLOW)

    def stop_reader(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        self.connect_btn.config(text="Connect")
        self.state_label.config(text="Not connected", fg=MUTED)

    def reset_view(self):
        for tile in (self.bpm_tile, self.spo2_tile, self.temp_tile, self.avg_tile):
            tile.set("--")
        self.bpm_graph.clear()
        self.temp_graph.clear()

    # ---------- recording ----------

    def toggle_record(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = self.csv_writer = None
            self.record_btn.config(text="Record CSV")
            self.add_log("Recording stopped")
            return
        name = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=datetime.now().strftime("heart_%Y-%m-%d_%H-%M.csv"),
            filetypes=[("CSV (Excel)", "*.csv")])
        if not name:
            return
        self.csv_file = open(name, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "cycle_second", "bpm", "spo2", "R", "PI_percent",
                                  "temp_C", "status"])
        self.record_btn.config(text="Stop recording")
        self.add_log(f"Recording to {name}")

    # ---------- reading and showing data ----------

    def poll(self):
        """Runs 10 times a second on the Tkinter thread and handles new lines."""
        try:
            while True:
                kind, text = self.lines.get_nowait()
                if kind == "line":
                    self.handle_line(text)
                elif kind == "info":
                    self.state_label.config(text=text, fg=GREEN)
                    self.add_log(text)
                elif kind == "error":
                    self.stop_reader()
                    self.state_label.config(text="Connection error", fg=RED)
                    self.add_log("ERROR: " + text)
                    if "denied" in text.lower() or "busy" in text.lower():
                        messagebox.showerror(
                            "Port busy",
                            "The port is being used by another program.\n\n"
                            "Close the Arduino IDE Serial Monitor (or Serial Plotter) and try again.")
        except queue.Empty:
            pass
        self.after(100, self.poll)

    def handle_line(self, line):
        if not line:
            return
        m = LIVE_RE.search(line)
        if m:
            self.show_live(*m.groups())
            return
        m = WAIT_RE.search(line)
        if m:
            temp = int(m.group(2))
            self.temp_tile.set(temp_text(temp), "air / object in front of sensor")
            self.temp_graph.add(temp / 10 if temp > -1000 else None)
            self.status_label.config(text="Put your finger on the sensor", fg=MUTED)
            self.clock_label.config(text="")
            return

        self.add_log(line)
        if line.startswith("--- 30s AVG"):
            self.in_avg_block = True
        elif self.in_avg_block:
            m = AVG_BPM_RE.search(line)
            if m:
                avg = int(m.group(1))
                self.avg_tile.set(str(avg) if avg > 0 else "--",
                                  "updated " + datetime.now().strftime("%H:%M:%S"))
            m = USED_RE.search(line)
            if m:
                self.avg_tile.note.config(
                    text=self.avg_tile.note.cget("text") + f"  ({m.group(1)} used, {m.group(2)} rejected)")
            if AVG_SPO2_RE.search(line):
                self.in_avg_block = False
        elif "FINGER DETECTED" in line:
            self.bpm_graph.clear()
        elif "FINGER REMOVED" in line:
            self.bpm_tile.set("--")
            self.spo2_tile.set("--")

    def show_live(self, sec, bpm, spo2, rx1000, pi, temp, status):
        bpm, spo2, rx1000, pi, temp = int(bpm), int(spo2), int(rx1000), int(pi), int(temp)

        if bpm > 0:
            normal = 60 <= bpm <= 100
            self.bpm_tile.set(str(bpm), "resting range 60-100" if normal else "outside 60-100 at rest",
                              RED if normal else YELLOW)
        else:
            self.bpm_tile.set("--", "measuring...")
        self.bpm_graph.add(bpm if bpm > 0 else None)

        if spo2 > 0:
            colour = BLUE if spo2 >= 95 else (YELLOW if spo2 >= 90 else RED)
            self.spo2_tile.set(str(spo2), f"R = {rx1000 / 1000:.3f}   PI = {pi / 100:.2f}%", colour)
        else:
            self.spo2_tile.set("--", "needs 5 good beats")

        self.temp_tile.set(temp_text(temp), "object / skin surface")
        self.temp_graph.add(temp / 10 if temp > -1000 else None)

        words, colour = STATUS_TEXT.get(status, (status, MUTED))
        self.status_label.config(text=words, fg=colour)
        self.clock_label.config(text=f"cycle {sec} s / 30 s")
        self.blink_heart()

        if self.csv_writer:
            self.csv_writer.writerow([datetime.now().strftime("%H:%M:%S"), sec, bpm or "",
                                      spo2 or "", rx1000 / 1000 if spo2 else "", pi / 100,
                                      temp_text(temp), status])
            self.csv_file.flush()

    def blink_heart(self):
        self.heart.config(fg=RED)
        self.after(180, lambda: self.heart.config(fg="#5a2a30"))

    def add_log(self, text):
        self.log.config(state="normal")
        self.log.insert("end", datetime.now().strftime("%H:%M:%S  ") + text + "\n")
        self.log.see("end")
        if int(self.log.index("end-1c").split(".")[0]) > 300:  # keep the log short
            self.log.delete("1.0", "100.0")
        self.log.config(state="disabled")

    def on_close(self):
        self.stop_reader()
        if self.csv_file:
            self.csv_file.close()
        self.destroy()


if __name__ == "__main__":
    Dashboard().mainloop()
