import struct
import threading
import time
import math
import queue
import serial
import tkinter as tk
from tkinter import ttk, messagebox

PORT = "COM4"      # <-- your Bluetooth SPP COM port
BAUD = 115200       # SPP often ignores baud, but keep consistent
READ_TIMEOUT = 0.2  # seconds

# Telemetry format: 3 float32 little-endian = 12 bytes
TELEM_FMT = "<fff"
TELEM_SIZE = struct.calcsize(TELEM_FMT)  # 12 bytes


def rx_loop(ser: serial.Serial, stop_flag: threading.Event, out_q: queue.Queue):
    """
    Continuously read telemetry frames from ESP32.
    Each frame is exactly 12 bytes: direction_deg, xw_m, yw_m (float32 LE).
    Pushes decoded tuples into out_q for the UI thread to consume.

    (Connection logic unchanged)
    """
    while not stop_flag.is_set():
        try:
            data = ser.read(TELEM_SIZE)
            if len(data) != TELEM_SIZE:
                continue

            direction_deg, x_m, y_m = struct.unpack(TELEM_FMT, data)
            out_q.put((direction_deg, x_m, y_m))

        except (serial.SerialException, OSError) as e:
            out_q.put(("__ERROR__", str(e), 0.0))
            break
        except struct.error:
            # Possible misalignment: flush and continue
            try:
                ser.reset_input_buffer()
            except Exception:
                pass

    out_q.put(("__STOP__", 0.0, 0.0))


class TelemetryUI:
    """
    Same connection logic as original,
    UI behavior matches the PyBluez version:
      - Amp +/- never greyed out (updates target even if disconnected)
      - D-pad knob draggable even if disconnected (only sends when connected)
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Snake — D-pad + Telemetry + Amp (+/-) (COM Port)")
        self.root.geometry("760x760")

        self.ser = None
        self.stop_flag = threading.Event()
        self.rx_thread = None
        self.q = queue.Queue()

        self.connected = False
        self.sending_lock = threading.Lock()

        # D-pad send throttling
        self.last_dpad_cmd = None
        self.last_cmd_time = 0.0
        self.hold_resend_sec = 0.15

        # Amp display (PC-side notion)
        self.amp_min = 0
        self.amp_max = 60
        self.amp_step = 5
        self.amp_target = 30

        # -------- UI Variables --------
        self.port_var = tk.StringVar(value=PORT)
        self.baud_var = tk.IntVar(value=BAUD)

        self.dir_var = tk.StringVar(value="—")
        self.x_var = tk.StringVar(value="—")
        self.y_var = tk.StringVar(value="—")
        self.last_rx_var = tk.StringVar(value="Never")

        self.status_var = tk.StringVar(value="Not connected.")

        # ===== Connection UI =====
        conn = ttk.LabelFrame(root, text="Connection (Serial / SPP COM port)")
        conn.pack(fill="x", padx=10, pady=10)

        ttk.Label(conn, text="COM Port:").grid(row=0, column=0, sticky="w", padx=8, pady=6)
        self.port_entry = ttk.Entry(conn, textvariable=self.port_var, width=16)
        self.port_entry.grid(row=0, column=1, sticky="w", padx=8, pady=6)

        ttk.Label(conn, text="Baud:").grid(row=0, column=2, sticky="w", padx=8, pady=6)
        self.baud_entry = ttk.Entry(conn, textvariable=self.baud_var, width=10)
        self.baud_entry.grid(row=0, column=3, sticky="w", padx=8, pady=6)

        self.btn_connect = ttk.Button(conn, text="Connect", command=self.connect)
        self.btn_connect.grid(row=1, column=1, sticky="ew", padx=8, pady=8)

        self.btn_disconnect = ttk.Button(conn, text="Disconnect", command=self.disconnect, state="disabled")
        self.btn_disconnect.grid(row=1, column=2, sticky="ew", padx=8, pady=8)

        conn.columnconfigure(1, weight=1)
        conn.columnconfigure(2, weight=1)

        ttk.Label(root, textvariable=self.status_var).pack(anchor="w", padx=12)

        # ===== Telemetry Panel =====
        telem = ttk.LabelFrame(root, text="Telemetry (ESP32 → PC, 12 bytes/frame)")
        telem.pack(fill="x", padx=10, pady=10)

        grid = ttk.Frame(telem)
        grid.pack(fill="x", padx=10, pady=10)

        ttk.Label(grid, text="Direction (deg):").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.dir_var, width=14).grid(row=0, column=1, sticky="w", padx=6, pady=4)

        ttk.Label(grid, text="X (cm):").grid(row=0, column=2, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.x_var, width=14).grid(row=0, column=3, sticky="w", padx=6, pady=4)

        ttk.Label(grid, text="Y (cm):").grid(row=0, column=4, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.y_var, width=14).grid(row=0, column=5, sticky="w", padx=6, pady=4)

        ttk.Label(grid, text="Last RX:").grid(row=1, column=0, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.last_rx_var).grid(row=1, column=1, columnspan=5, sticky="w", padx=6, pady=4)

        for c in range(6):
            grid.columnconfigure(c, weight=1)

        # ===== Amplitude controls =====
        amp_frame = ttk.LabelFrame(root, text="Amplitude (sends 0..9, +, - to ESP32)")
        amp_frame.pack(fill="x", padx=10, pady=10)

        row = ttk.Frame(amp_frame)
        row.pack(fill="x", padx=10, pady=10)

        self.amp_var = tk.StringVar(value=str(self.amp_target))
        ttk.Label(row, text="Target Amp (deg):").pack(side="left")
        ttk.Label(row, textvariable=self.amp_var, width=6).pack(side="left", padx=(6, 20))

        # NOTE: NOT greyed out anymore (matches PyBluez UI)
        self.amp_minus_btn = ttk.Button(row, text="Amp -", command=self._amp_minus)
        self.amp_minus_btn.pack(side="left", padx=6)

        self.amp_plus_btn = ttk.Button(row, text="Amp +", command=self._amp_plus)
        self.amp_plus_btn.pack(side="left", padx=6)

        ttk.Label(
            amp_frame,
            text="Keyboard: '+' / '-' (also keypad +/-). Digits 0..9 set amp to digit*5 deg.",
        ).pack(anchor="w", padx=10, pady=(0, 10))

        # ===== D-pad =====
        dpad_frame = ttk.LabelFrame(root, text="D-pad (movement control)")
        dpad_frame.pack(fill="x", padx=10, pady=10)

        ttk.Label(
            dpad_frame,
            text=(
                "Drag mapping:\n"
                "  • Drag LEFT  → 'l' (turn left)\n"
                "  • Drag RIGHT → 'r' (turn right)\n"
                "  • Drag UP    → 's' (straight/run)\n"
                "  • Drag DOWN  → 'c' (calibrate+STOP)\n"
                "  • Middle (deadzone) → does nothing\n"
                "Release → snaps to center, does nothing."
            ),
            justify="left",
        ).pack(anchor="w", padx=10, pady=(6, 0))

        self.canvas = tk.Canvas(dpad_frame, width=320, height=320, highlightthickness=0)
        self.canvas.pack(padx=10, pady=10)

        # Geometry
        self.cx, self.cy = 160, 160
        self.base_r = 130
        self.knob_r = 22
        self.deadzone_px = 25

        self.canvas.create_oval(
            self.cx - self.base_r, self.cy - self.base_r,
            self.cx + self.base_r, self.cy + self.base_r,
            outline="#999", width=3
        )

        self._draw_arrows()

        self.knob = self.canvas.create_oval(
            self.cx - self.knob_r, self.cy - self.knob_r,
            self.cx + self.knob_r, self.cy + self.knob_r,
            fill="#ddd", outline="#666", width=2
        )

        self.dragging = False
        # Use ButtonPress-1 for best consistency
        self.canvas.bind("<ButtonPress-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

        # ===== Log =====
        log_frame = ttk.LabelFrame(root, text="Log")
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.log = tk.Text(log_frame, height=10, wrap="word")
        self.log.pack(fill="both", expand=True, padx=8, pady=8)
        self.log.configure(state="disabled")

        # Key binds
        self.root.bind("<Escape>", lambda e: self.disconnect())
        self.root.bind("<Key>", self.on_key)

        # Poll telemetry
        self.root.after(50, self._poll_queue)

        # Proper close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- Logging ----------------
    def append_log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _set_connected_ui(self, connected: bool):
        self.connected = connected
        self.btn_connect.config(state="disabled" if connected else "normal")
        self.btn_disconnect.config(state="normal" if connected else "disabled")
        if not connected:
            self.last_dpad_cmd = None
            self.last_cmd_time = 0.0

    # ---------------- Connection logic (UNCHANGED) ----------------
    def connect(self):
        if self.ser is not None:
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Please enter a COM port (e.g., COM11).")
            return

        try:
            self.ser = serial.Serial(port, BAUD, timeout=READ_TIMEOUT)
            time.sleep(0.2)
        except Exception as e:
            self.ser = None
            messagebox.showerror("Connection Failed", f"Could not open {port}\n\n{e}")
            self._set_connected_ui(False)
            self.status_var.set("Not connected.")
            return

        self.stop_flag.clear()
        self.rx_thread = threading.Thread(target=rx_loop, args=(self.ser, self.stop_flag, self.q), daemon=True)
        self.rx_thread.start()

        self.status_var.set(f"Connected to {port}")
        self.append_log(f"Connected to {port}")
        self._set_connected_ui(True)

    def disconnect(self):
        if self.ser is None:
            return

        self.append_log("Disconnecting...")
        self.stop_flag.set()
        time.sleep(0.1)

        try:
            self.ser.close()
        except Exception:
            pass

        self.ser = None
        self.status_var.set("Not connected.")
        self.append_log("Disconnected.")
        self._set_connected_ui(False)

    # ---------------- Sending ----------------
    def send_one_byte(self, c: str, log: bool = True):
        if self.ser is None:
            return
        try:
            self.ser.write(c.encode("utf-8"))
            if log:
                self.append_log(f"Sent cmd: {c}")
        except Exception as e:
            messagebox.showerror("Send Failed", str(e))

    def _send_cmd_char(self, ch: str):
        # Match PyBluez UI behavior: if not connected, just do nothing
        if not self.connected or not self.ser:
            return
        if not ch or len(ch) != 1:
            return

        dpad_cmds = {'l', 'r', 's', 'c'}
        now = time.time()
        if ch in dpad_cmds:
            if ch == self.last_dpad_cmd and (now - self.last_cmd_time) < self.hold_resend_sec:
                return
            self.last_dpad_cmd = ch
            self.last_cmd_time = now

        def worker():
            with self.sending_lock:
                self.send_one_byte(ch, log=True)

        threading.Thread(target=worker, daemon=True).start()

    # ---------------- Amplitude helpers ----------------
    def _set_amp_target(self, amp: int):
        amp = max(self.amp_min, min(self.amp_max, amp))
        self.amp_target = amp
        self.amp_var.set(str(self.amp_target))

    def _amp_plus(self):
        # Update display regardless of connection (matches PyBluez UI feel)
        self._set_amp_target(self.amp_target + self.amp_step)
        # Send only if connected
        self._send_cmd_char('+')

    def _amp_minus(self):
        self._set_amp_target(self.amp_target - self.amp_step)
        self._send_cmd_char('-')

    def _amp_digit(self, d: int):
        self._set_amp_target(d * 5)
        self._send_cmd_char(str(d))

    # ---------------- Key handling ----------------
    def on_key(self, event: tk.Event):
        ch = event.char
        if ch == '+':
            self._amp_plus()
            return
        if ch == '-':
            self._amp_minus()
            return
        if ch.isdigit():
            self._amp_digit(int(ch))
            return

        ks = getattr(event, "keysym", "")
        if ks in ("KP_Add", "plus"):
            self._amp_plus()
            return
        if ks in ("KP_Subtract", "minus"):
            self._amp_minus()
            return

    # ---------------- Receiving telemetry ----------------
    def _poll_queue(self):
        try:
            while True:
                item = self.q.get_nowait()

                if isinstance(item, tuple) and len(item) == 3 and item[0] == "__ERROR__":
                    self.status_var.set(f"Serial error: {item[1]}")
                    self.append_log(f"Serial error: {item[1]}")
                    self.disconnect()
                    break

                if isinstance(item, tuple) and len(item) == 3 and item[0] == "__STOP__":
                    break

                direction_deg, x_m, y_m = item
                self.dir_var.set(f"{direction_deg:0.2f}")
                self.x_var.set(f"{x_m:0.4f}")
                self.y_var.set(f"{y_m:0.4f}")
                self.last_rx_var.set(time.strftime("%H:%M:%S"))

        except queue.Empty:
            pass

        self.root.after(50, self._poll_queue)

    # ---------------- D-pad drawing ----------------
    def _draw_arrows(self):
        arrow_r_outer = self.base_r - 10
        arrow_r_inner = self.base_r - 45
        w = 22

        def tri(points, tag):
            return self.canvas.create_polygon(points, fill="#f2f2f2", outline="#777", width=2, tags=(tag,))

        tri([(self.cx, self.cy - arrow_r_outer),
             (self.cx - w, self.cy - arrow_r_inner),
             (self.cx + w, self.cy - arrow_r_inner)], "arrow_up")

        tri([(self.cx + arrow_r_outer, self.cy),
             (self.cx + arrow_r_inner, self.cy - w),
             (self.cx + arrow_r_inner, self.cy + w)], "arrow_right")

        tri([(self.cx, self.cy + arrow_r_outer),
             (self.cx - w, self.cy + arrow_r_inner),
             (self.cx + w, self.cy + arrow_r_inner)], "arrow_down")

        tri([(self.cx - arrow_r_outer, self.cy),
             (self.cx - arrow_r_inner, self.cy - w),
             (self.cx - arrow_r_inner, self.cy + w)], "arrow_left")

        # Clicking arrows moves knob regardless; sends only if connected
        self.canvas.tag_bind("arrow_up", "<Button-1>", lambda e: self._arrow_click("up"))
        self.canvas.tag_bind("arrow_right", "<Button-1>", lambda e: self._arrow_click("right"))
        self.canvas.tag_bind("arrow_down", "<Button-1>", lambda e: self._arrow_click("down"))
        self.canvas.tag_bind("arrow_left", "<Button-1>", lambda e: self._arrow_click("left"))

    def _arrow_click(self, direction: str):
        if direction == "left":
            self._set_knob_from_normalized(-1.0, 0.0)
            self._send_cmd_char('l')
        elif direction == "right":
            self._set_knob_from_normalized(1.0, 0.0)
            self._send_cmd_char('r')
        elif direction == "up":
            self._set_knob_from_normalized(0.0, 1.0)
            self._send_cmd_char('s')
        elif direction == "down":
            self._set_knob_from_normalized(0.0, -1.0)
            self._send_cmd_char('c')

    # ---------------- D-pad interactions ----------------
    def on_canvas_click(self, event):
        dx, dy = event.x - self.cx, event.y - self.cy
        if math.hypot(dx, dy) <= self.base_r:
            self.dragging = True
            self._update_stick(event.x, event.y, send_now=True)

    def on_canvas_drag(self, event):
        if not self.dragging:
            return
        self._update_stick(event.x, event.y, send_now=True)

    def on_canvas_release(self, event):
        if not self.dragging:
            return
        self.dragging = False
        self._set_knob_center()
        self.last_dpad_cmd = None

    def _set_knob_center(self):
        self.canvas.coords(
            self.knob,
            self.cx - self.knob_r, self.cy - self.knob_r,
            self.cx + self.knob_r, self.cy + self.knob_r
        )

    def _set_knob_from_normalized(self, x_norm: float, y_norm: float):
        max_d = self.base_r - self.knob_r - 6
        x_norm = max(-1.0, min(1.0, x_norm))
        y_norm = max(-1.0, min(1.0, y_norm))
        kx = self.cx + x_norm * max_d
        ky = self.cy - y_norm * max_d
        self.canvas.coords(
            self.knob,
            kx - self.knob_r, ky - self.knob_r,
            kx + self.knob_r, ky + self.knob_r
        )

    def _update_stick(self, x, y, send_now: bool):
        dx = x - self.cx
        dy = y - self.cy
        dist = math.hypot(dx, dy)

        max_d = self.base_r - self.knob_r - 6
        if dist > max_d and dist > 1e-6:
            scale = max_d / dist
            dx *= scale
            dy *= scale

        kx = self.cx + dx
        ky = self.cy + dy
        self.canvas.coords(
            self.knob,
            kx - self.knob_r, ky - self.knob_r,
            kx + self.knob_r, ky + self.knob_r
        )

        if not send_now:
            return

        if math.hypot(dx, dy) < self.deadzone_px:
            return

        if abs(dx) > abs(dy):
            cmd = 'l' if dx < 0 else 'r'
        else:
            cmd = 's' if dy < 0 else 'c'

        self._send_cmd_char(cmd)

    # ---------------- Close ----------------
    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass

    TelemetryUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
