import struct
import threading
import time
import queue
import serial
import tkinter as tk
from tkinter import ttk, messagebox

# =========================
# Defaults (edit)
# =========================
DEFAULT_PORT = "COM11"   # Windows Bluetooth SPP OUTGOING COM port
BAUD = 115200
READ_TIMEOUT = 0.2

# Telemetry: 3 float32 little-endian = 12 bytes
TELEM_FMT = "<fff"
TELEM_SIZE = struct.calcsize(TELEM_FMT)  # 12 bytes
RECV_CHUNK = 1024


def rx_loop(ser: serial.Serial, stop_flag: threading.Event, out_q: queue.Queue):
    """
    Read raw bytes from serial and parse fixed 12-byte telemetry frames (<fff).
    Push tuples (direction_deg, x_m, y_m) into out_q for UI thread.

    If alignment breaks (rare), we flush input buffer and continue.
    """
    buf = bytearray()
    out_q.put(("__LOG__", "RX thread started."))

    while not stop_flag.is_set():
        try:
            chunk = ser.read(RECV_CHUNK)  # b"" on timeout
            if not chunk:
                continue

            buf.extend(chunk)

            while len(buf) >= TELEM_SIZE:
                frame = bytes(buf[:TELEM_SIZE])
                del buf[:TELEM_SIZE]

                try:
                    direction_deg, x_m, y_m = struct.unpack(TELEM_FMT, frame)
                    out_q.put((direction_deg, x_m, y_m))
                except struct.error:
                    # Possible misalignment: flush and resync
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    buf.clear()
                    out_q.put(("__LOG__", "Unpack misalignment -> flushed input buffer (resync)."))
                    break

        except (serial.SerialException, OSError) as e:
            out_q.put(("__ERROR__", str(e), 0.0))
            break

    out_q.put(("__STOP__", 0.0, 0.0))
    out_q.put(("__LOG__", "RX thread stopped."))


class TelemetryControlUI:
    """
    Classic Bluetooth SPP over COM port:
      - RX: telemetry frames (12 bytes) from ESP32
      - TX: single-char commands: c l r s 0..9 + -
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Snake — SPP COM Telemetry + Control")
        self.root.geometry("760x620")

        self.ser: serial.Serial | None = None
        self.stop_flag = threading.Event()
        self.rx_thread = None
        self.q: queue.Queue = queue.Queue()

        # D-pad send throttling (avoid spamming)
        self.last_dpad_cmd = None
        self.last_cmd_time = 0.0
        self.hold_resend_sec = 0.15

        # PC-side amplitude display (optional)
        self.amp_target = 30
        self.amp_var = tk.StringVar(value=str(self.amp_target))

        # -------- UI Variables --------
        self.port_var = tk.StringVar(value=DEFAULT_PORT)

        self.dir_var = tk.StringVar(value="--")
        self.x_var = tk.StringVar(value="--")
        self.y_var = tk.StringVar(value="--")
        self.last_rx_var = tk.StringVar(value="Never")

        self.status_var = tk.StringVar(value="Disconnected")

        # -------- Layout --------
        pad = {"padx": 10, "pady": 8}

        # Connection
        conn = ttk.LabelFrame(root, text="Connection (Bluetooth SPP COM Port)")
        conn.pack(fill="x", **pad)

        ttk.Label(conn, text="COM Port (Outgoing):").grid(row=0, column=0, sticky="w", padx=8, pady=6)
        self.port_entry = ttk.Entry(conn, textvariable=self.port_var, width=12)
        self.port_entry.grid(row=0, column=1, sticky="w", padx=8, pady=6)

        self.btn_connect = ttk.Button(conn, text="Connect", command=self.connect)
        self.btn_connect.grid(row=0, column=2, padx=8, pady=6)

        self.btn_disconnect = ttk.Button(conn, text="Disconnect", command=self.disconnect, state="disabled")
        self.btn_disconnect.grid(row=0, column=3, padx=8, pady=6)

        conn.columnconfigure(4, weight=1)

        ttk.Label(root, textvariable=self.status_var).pack(anchor="w", padx=12)

        # Telemetry
        telem = ttk.LabelFrame(root, text="Telemetry (ESP32 → PC, 12 bytes/frame: <fff)")
        telem.pack(fill="x", **pad)

        grid = ttk.Frame(telem)
        grid.pack(fill="x", padx=10, pady=10)
        grid.columnconfigure(1, weight=1)

        self._make_row(grid, "Direction (deg):", self.dir_var, row=0)
        self._make_row(grid, "Pos X (m):", self.x_var, row=1)
        self._make_row(grid, "Pos Y (m):", self.y_var, row=2)

        ttk.Label(grid, text="Last RX:").grid(row=3, column=0, sticky="w", pady=6)
        e = ttk.Entry(grid, textvariable=self.last_rx_var, state="readonly", width=20, justify="center")
        e.grid(row=3, column=1, sticky="ew", padx=(10, 0), pady=6)

        # Controls
        ctrl = ttk.LabelFrame(root, text="Controls (PC → ESP32, single-char)")
        ctrl.pack(fill="x", **pad)

        hint = (
            "Valid commands: c l r s 0..9 + -\n"
            "• D-pad drag: LEFT='l', RIGHT='r', UP='s', DOWN='c'\n"
            "• Keyboard: '+' / '-' adjust amplitude; digits 0..9 set amplitude (digit*5)\n"
            "• Manual command box sends first character only"
        )
        ttk.Label(ctrl, text=hint, justify="left").pack(anchor="w", padx=10, pady=(6, 0))

        row = ttk.Frame(ctrl)
        row.pack(fill="x", padx=10, pady=10)
        row.columnconfigure(1, weight=1)

        ttk.Label(row, text="Manual command:").grid(row=0, column=0, sticky="w")
        self.cmd_entry = ttk.Entry(row)
        self.cmd_entry.grid(row=0, column=1, sticky="ew", padx=(6, 6))
        self.cmd_entry.bind("<Return>", lambda e: self.send_manual_command())
        self.btn_send = ttk.Button(row, text="Send", command=self.send_manual_command, state="disabled")
        self.btn_send.grid(row=0, column=2, sticky="e")

        # Amp buttons (optional)
        amp_row = ttk.Frame(ctrl)
        amp_row.pack(fill="x", padx=10, pady=(0, 10))
        ttk.Label(amp_row, text="Amp target (PC display):").pack(side="left")
        ttk.Label(amp_row, textvariable=self.amp_var, width=6).pack(side="left", padx=(6, 20))
        self.btn_amp_minus = ttk.Button(amp_row, text="Amp -", command=lambda: self.send_command('-'), state="disabled")
        self.btn_amp_minus.pack(side="left", padx=6)
        self.btn_amp_plus = ttk.Button(amp_row, text="Amp +", command=lambda: self.send_command('+'), state="disabled")
        self.btn_amp_plus.pack(side="left", padx=6)

        # D-pad (drag)
        dpad = ttk.LabelFrame(root, text="D-pad (drag only)")
        dpad.pack(fill="x", **pad)

        self.canvas = tk.Canvas(dpad, width=320, height=320, highlightthickness=0)
        self.canvas.pack(padx=10, pady=10)

        self.cx, self.cy = 160, 160
        self.base_r = 130
        self.knob_r = 22
        self.deadzone_px = 25

        self.canvas.create_oval(
            self.cx - self.base_r, self.cy - self.base_r,
            self.cx + self.base_r, self.cy + self.base_r,
            outline="#999", width=3
        )

        self.knob = self.canvas.create_oval(
            self.cx - self.knob_r, self.cy - self.knob_r,
            self.cx + self.knob_r, self.cy + self.knob_r,
            fill="#ddd", outline="#666", width=2
        )

        self.dragging = False
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

        # Log
        logf = ttk.LabelFrame(root, text="Log")
        logf.pack(fill="both", expand=True, padx=10, pady=10)

        self.log = tk.Text(logf, height=10, wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True, padx=8, pady=8)

        # Poll queue for telemetry/log/errors
        self.root.after(50, self._poll_queue)

        # Keybinds
        root.bind("<Escape>", lambda e: self.disconnect())
        root.bind("<Key>", self.on_key)

        # Close behavior
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _make_row(self, parent, label, var, row: int):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=6)
        e = ttk.Entry(parent, textvariable=var, state="readonly", width=20, justify="center")
        e.grid(row=row, column=1, sticky="ew", padx=(10, 0), pady=6)

    def _log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    # ---------------- Connection ----------------

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
            return

        self.stop_flag.clear()
        self.rx_thread = threading.Thread(target=rx_loop, args=(self.ser, self.stop_flag, self.q), daemon=True)
        self.rx_thread.start()

        self.status_var.set(f"Connected to {port}")
        self.btn_connect.config(state="disabled")
        self.btn_disconnect.config(state="normal")
        self.btn_send.config(state="normal")
        self.btn_amp_minus.config(state="normal")
        self.btn_amp_plus.config(state="normal")
        self.cmd_entry.focus_set()

        self._log(f"Opened serial: {port} @ {BAUD}")

    def disconnect(self):
        if self.ser is None:
            return

        self.stop_flag.set()
        time.sleep(0.1)

        try:
            self.ser.close()
        except Exception:
            pass

        self.ser = None
        self.status_var.set("Disconnected")
        self.btn_connect.config(state="normal")
        self.btn_disconnect.config(state="disabled")
        self.btn_send.config(state="disabled")
        self.btn_amp_minus.config(state="disabled")
        self.btn_amp_plus.config(state="disabled")

        self._log("Disconnected.")

    # ---------------- Sending ----------------

    def send_command(self, c: str):
        """
        Send exactly one ASCII command character to ESP32.
        """
        if self.ser is None:
            return
        if not c or len(c) != 1:
            return

        valid = "lrs c0123456789+-".replace(" ", "")
        if c not in valid:
            self._log(f"Ignored invalid cmd: {repr(c)}")
            return

        # Throttle only for continuous D-pad commands
        dpad_cmds = {'l', 'r', 's', 'c'}
        now = time.time()
        if c in dpad_cmds:
            if c == self.last_dpad_cmd and (now - self.last_cmd_time) < self.hold_resend_sec:
                return
            self.last_dpad_cmd = c
            self.last_cmd_time = now

        try:
            self.ser.write(c.encode("ascii"))
            self.status_var.set(f"Sent: {c}")
            self._log(f"Sent: {c}")
        except Exception as e:
            messagebox.showerror("Send Failed", str(e))
            self._log(f"Send failed: {e}")

        # Update PC-side amplitude display (optional)
        if c.isdigit():
            self.amp_target = int(c) * 5
            self.amp_var.set(str(self.amp_target))
        elif c == '+':
            self.amp_target = min(60, self.amp_target + 5)
            self.amp_var.set(str(self.amp_target))
        elif c == '-':
            self.amp_target = max(0, self.amp_target - 5)
            self.amp_var.set(str(self.amp_target))

    def send_manual_command(self):
        text = self.cmd_entry.get().strip()
        if not text:
            return
        self.send_command(text[0])
        self.cmd_entry.delete(0, "end")

    def on_key(self, event):
        # Only handle amplitude keys + digits here; D-pad is via mouse drag
        ch = event.char
        if ch in ['+', '-']:
            self.send_command(ch)
        elif ch.isdigit():
            self.send_command(ch)

    # ---------------- Telemetry queue ----------------

    def _poll_queue(self):
        try:
            while True:
                item = self.q.get_nowait()

                if isinstance(item, tuple) and len(item) == 3 and item[0] == "__ERROR__":
                    self.status_var.set(f"Serial error: {item[1]}")
                    self._log(f"Serial error: {item[1]}")
                    self.disconnect()
                    break

                if isinstance(item, tuple) and len(item) == 3 and item[0] == "__STOP__":
                    break

                if isinstance(item, tuple) and len(item) == 2 and item[0] == "__LOG__":
                    self._log(str(item[1]))
                    continue

                # Telemetry tuple
                direction_deg, x_m, y_m = item
                self.dir_var.set(f"{direction_deg:0.2f}")
                self.x_var.set(f"{x_m:0.4f}")
                self.y_var.set(f"{y_m:0.4f}")
                self.last_rx_var.set(time.strftime("%H:%M:%S"))

        except queue.Empty:
            pass

        self.root.after(50, self._poll_queue)

    # ---------------- D-pad interactions ----------------

    def on_canvas_click(self, event):
        dx, dy = event.x - self.cx, event.y - self.cy
        if math.hypot(dx, dy) <= self.base_r:
            self.dragging = True
            self._update_stick(event.x, event.y)

    def on_canvas_drag(self, event):
        if not self.dragging:
            return
        self._update_stick(event.x, event.y)

    def on_canvas_release(self, event):
        if not self.dragging:
            return
        self.dragging = False
        self.canvas.coords(
            self.knob,
            self.cx - self.knob_r, self.cy - self.knob_r,
            self.cx + self.knob_r, self.cy + self.knob_r
        )
        # allow next drag to re-send immediately
        self.last_dpad_cmd = None

    def _update_stick(self, x, y):
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

        # deadzone: do nothing
        if math.hypot(dx, dy) < self.deadzone_px:
            return

        # dominant axis decision
        if abs(dx) > abs(dy):
            cmd = 'l' if dx < 0 else 'r'
        else:
            cmd = 's' if dy < 0 else 'c'

        self.send_command(cmd)

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

    TelemetryControlUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
