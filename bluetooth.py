import struct
import threading
import time
import math
import queue
import serial
import tkinter as tk
from tkinter import ttk, messagebox

PORT = "COM11"      # <-- your Bluetooth SPP COM port
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
    UI:
      - Amp +/- never greyed out (updates target even if disconnected)
      - D-pad draggable even if disconnected (only sends when connected)

    TX protocol (ALL sends are exactly 8 bytes ASCII):
      1) Mode markers (1 letter + 7 zeros):
         - Manual:    "M0000000"
         - Direction: "D0000000"  (auto angle)
         - Position:  "P0000000"  (auto coords)

      2) Auto payload values (NO leading letter anymore):
         - Angle value: "00000120"   (8 digits)
         - X value:     "00000100"
         - Y value:     "00000200"

      3) D-pad and amp commands remain 1-char header + 7 zeros:
         - 'l','r','s','c','+','-','0'..'9' -> e.g. "l0000000", "+0000000", "70000000"
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Snake — D-pad + Telemetry + Amp (+/-) + Modes (8B cmds)")
        self.root.geometry("900x780")

        self.ser = None
        self.stop_flag = threading.Event()
        self.rx_thread = None
        self.q = queue.Queue()

        self.connected = False
        self.sending_lock = threading.Lock()

        # D-pad send throttling (still applies even with 8B frames)
        self.last_dpad_cmd = None
        self.last_cmd_time = 0.0
        self.hold_resend_sec = 0.15

        # Manual analog steering send throttling
        self.last_manual_angle = None
        self.last_manual_angle_time = 0.0
        self.manual_angle_resend_sec = 0.08
        self.manual_angle_step_deg = 2

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

        # =========================
        # Amp + Mode panels (side-by-side)
        # =========================
        top_controls = ttk.Frame(root)
        top_controls.pack(fill="x", padx=10, pady=10)
        top_controls.columnconfigure(0, weight=1)
        top_controls.columnconfigure(1, weight=1)

        # ===== Amplitude controls =====
        amp_frame = ttk.LabelFrame(top_controls, text="Amplitude (sends 0..9, +, -) — 8B frames")
        amp_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        row = ttk.Frame(amp_frame)
        row.pack(fill="x", padx=10, pady=10)

        self.amp_var = tk.StringVar(value=str(self.amp_target))
        ttk.Label(row, text="Target Amp (deg):").pack(side="left")
        ttk.Label(row, textvariable=self.amp_var, width=6).pack(side="left", padx=(6, 20))

        self.amp_minus_btn = ttk.Button(row, text="Amp -", command=self._amp_minus)
        self.amp_minus_btn.pack(side="left", padx=6)

        self.amp_plus_btn = ttk.Button(row, text="Amp +", command=self._amp_plus)
        self.amp_plus_btn.pack(side="left", padx=6)

        ttk.Label(
            amp_frame,
            text="Keyboard: '+' / '-' (also keypad +/-). Digits 0..9 set amp to digit*5 deg.",
        ).pack(anchor="w", padx=10, pady=(0, 10))

        # ===== Mode panel =====
        mode_frame = ttk.LabelFrame(top_controls, text="Control Mode (Manual / Automatic) — 8B frames")
        mode_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        self.control_mode_var = tk.StringVar(value="manual")  # "manual" or "auto"
        self.auto_submode_var = tk.StringVar(value="angle")   # "angle" or "coords"

        mode_row = ttk.Frame(mode_frame)
        mode_row.pack(fill="x", padx=10, pady=(10, 6))

        ttk.Radiobutton(
            mode_row, text="Manual (D-pad)", value="manual",
            variable=self.control_mode_var, command=self._on_mode_change
        ).pack(side="left", padx=(0, 16))

        ttk.Radiobutton(
            mode_row, text="Automatic", value="auto",
            variable=self.control_mode_var, command=self._on_mode_change
        ).pack(side="left")

        ttk.Separator(mode_frame).pack(fill="x", padx=10, pady=8)

        sub_row = ttk.Frame(mode_frame)
        sub_row.pack(fill="x", padx=10, pady=(0, 6))

        ttk.Label(sub_row, text="Auto sub-mode:").pack(side="left", padx=(0, 8))

        self.rb_angle = ttk.Radiobutton(
            sub_row, text="Angle", value="angle",
            variable=self.auto_submode_var, command=self._on_auto_submode_change
        )
        self.rb_angle.pack(side="left", padx=(0, 10))

        self.rb_coords = ttk.Radiobutton(
            sub_row, text="Coords", value="coords",
            variable=self.auto_submode_var, command=self._on_auto_submode_change
        )
        self.rb_coords.pack(side="left")

        # Angle controls (now sends ONLY 8-digit value)
        self.angle_box = ttk.Frame(mode_frame)
        self.angle_box.pack(fill="x", padx=10, pady=(6, 0))

        angle_in = ttk.Frame(self.angle_box)
        angle_in.pack(fill="x")

        ttk.Label(angle_in, text="Angle (deg):").pack(side="left")
        self.angle_var = tk.StringVar(value="120")
        self.angle_entry = ttk.Entry(angle_in, textvariable=self.angle_var, width=10)
        self.angle_entry.pack(side="left", padx=8)

        self.btn_send_angle = ttk.Button(angle_in, text="Send 8-digit value", command=self._send_angle)
        self.btn_send_angle.pack(side="left")

        ttk.Label(
            self.angle_box,
            text="Sends: 8 digits only (example: 00000120). Mode marker already sent: D0000000",
        ).pack(anchor="w", pady=(6, 0))

        # Coords controls (now sends ONLY 8-digit value per X/Y)
        self.coords_box = ttk.Frame(mode_frame)
        self.coords_box.pack(fill="x", padx=10, pady=(10, 0))

        coords_in1 = ttk.Frame(self.coords_box)
        coords_in1.pack(fill="x", pady=(0, 6))

        ttk.Label(coords_in1, text="X:").pack(side="left")
        self.xcmd_var = tk.StringVar(value="100")
        self.xcmd_entry = ttk.Entry(coords_in1, textvariable=self.xcmd_var, width=10)
        self.xcmd_entry.pack(side="left", padx=8)
        self.btn_send_x = ttk.Button(coords_in1, text="Send 8-digit X", command=self._send_x)
        self.btn_send_x.pack(side="left")

        coords_in2 = ttk.Frame(self.coords_box)
        coords_in2.pack(fill="x")

        ttk.Label(coords_in2, text="Y:").pack(side="left")
        self.ycmd_var = tk.StringVar(value="200")
        self.ycmd_entry = ttk.Entry(coords_in2, textvariable=self.ycmd_var, width=10)
        self.ycmd_entry.pack(side="left", padx=8)
        self.btn_send_y = ttk.Button(coords_in2, text="Send 8-digit Y", command=self._send_y)
        self.btn_send_y.pack(side="left")

        self.btn_send_xy = ttk.Button(self.coords_box, text="Send X then Y", command=self._send_xy)
        self.btn_send_xy.pack(anchor="w", pady=(8, 0))

        ttk.Label(
            mode_frame,
            text="Sends: 8 digits only (examples: 00000100, 00000200). Mode marker already sent: P0000000",
        ).pack(anchor="w", padx=10, pady=(10, 10))

        # ===== D-pad =====
        dpad_frame = ttk.LabelFrame(root, text="D-pad (manual analog steering) — angle -90..90")
        dpad_frame.pack(fill="both", expand=True, padx=10, pady=10)

        dpad_info = ttk.Frame(dpad_frame)
        dpad_info.pack(fill="x", padx=10, pady=(6, 0))

        ttk.Label(dpad_info, text="Manual angle (deg):").pack(side="left")
        self.manual_angle_var = tk.StringVar(value="0")
        ttk.Label(dpad_info, textvariable=self.manual_angle_var, width=6).pack(side="left", padx=(6, 0))

        self.canvas = tk.Canvas(dpad_frame, width=320, height=320, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self.canvas.bind("<Configure>", self._on_canvas_resize)

        # Geometry
        self.cx, self.cy = 160, 160
        self.base_r = 130
        self.knob_r = 22
        self.deadzone_px = 25

        self.dpad_enabled = True  # toggled by mode switch

        self.base_circle = self.canvas.create_oval(
            self.cx - self.base_r, self.cy - self.base_r,
            self.cx + self.base_r, self.cy + self.base_r,
            outline="#999", width=3
        )

        self.arrow_ids = []
        self._draw_arrows()

        self.knob = self.canvas.create_oval(
            self.cx - self.knob_r, self.cy - self.knob_r,
            self.cx + self.knob_r, self.cy + self.knob_r,
            fill="#ddd", outline="#666", width=2
        )

        self.dragging = False
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

        # Initialize mode UI state
        self._on_mode_change()

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

    # ---------------- Connection logic ----------------
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

    # ---------------- 8-byte sending helpers ----------------
    def _send_8b_head(self, head: str, value7: int = 0, log: bool = True):
        """
        Send exactly 8 bytes:
          [1 ASCII char][7 ASCII digits]
        Used for mode markers and 1-char commands (dpad, +/-, digits).
        Examples:
          M0000000, D0000000, P0000000, l0000000, +0000000, 70000000
        """
        if not self.connected or not self.ser:
            return
        if not head or len(head) != 1:
            return

        try:
            v = int(value7)
        except Exception:
            v = 0
        v = max(0, min(9_999_999, v))

        frame = f"{head}{v:07d}"
        payload = frame.encode("ascii")  # exactly 8 bytes

        def worker():
            with self.sending_lock:
                try:
                    self.ser.write(payload)
                    if log:
                        self.append_log(f"Sent 8B cmd: {frame}")
                except Exception as e:
                    messagebox.showerror("Send Failed", str(e))

        threading.Thread(target=worker, daemon=True).start()

    def _send_8b_number(self, value: int, log: bool = True):
        """
        Send exactly 8 bytes, digits-only, custom format:
        120 -> "01200000"
        5   -> "00500000"
        100 -> "01000000"
        Rule: take the number as 3 digits (zero-filled), prefix '0', then pad right with zeros to 8.
        """
        if not self.connected or not self.ser:
            return

        try:
            v = int(value)
        except Exception:
            v = 0

        # keep it sane; this encoding only preserves the first 3 digits anyway
        v = max(0, min(999, v))

        s3 = str(v).zfill(3)           # 120 -> "120", 5 -> "005"
        frame = ("0" + s3 + "0000")[:8]  # -> "01200000"
        payload = frame.encode("ascii")  # exactly 8 bytes

        def worker():
            with self.sending_lock:
                try:
                    self.ser.write(payload)
                    if log:
                        self.append_log(f"Sent 8B num: {frame}")
                except Exception as e:
                    messagebox.showerror("Send Failed", str(e))

        threading.Thread(target=worker, daemon=True).start()

    def _send_cmd_char(self, ch: str):
        """
        For 1-char commands (dpad, +/-, digits), send as head+7 zeros.
        """
        if not ch or len(ch) != 1:
            return

        dpad_cmds = {'l', 'r', 's', 'c'}
        now = time.time()
        if ch in dpad_cmds:
            if ch == self.last_dpad_cmd and (now - self.last_cmd_time) < self.hold_resend_sec:
                return
            self.last_dpad_cmd = ch
            self.last_cmd_time = now

        self._send_8b_head(ch, 0, log=True)

    def _send_manual_angle(self, angle_deg: float, force: bool = False):
        """
        Manual analog steering angle: -90..90 deg.
                Sent as 8-byte numeric payload using reversed 0..180 encoding:
                    tx_value = 90 - angle_deg
                Example: -90 -> 180, 0 -> 90, 90 -> 0
        """
        angle = int(round(max(-90, min(90, angle_deg))))
        self.manual_angle_var.set(str(angle))

        now = time.time()
        if not force and self.last_manual_angle is not None:
            if abs(angle - self.last_manual_angle) < self.manual_angle_step_deg:
                if (now - self.last_manual_angle_time) < self.manual_angle_resend_sec:
                    return

        tx_value = 90 - angle
        self.last_manual_angle = angle
        self.last_manual_angle_time = now
        self._send_8b_number(tx_value, log=False)

    # ---------------- Amplitude helpers ----------------
    def _set_amp_target(self, amp: int):
        amp = max(self.amp_min, min(self.amp_max, amp))
        self.amp_target = amp
        self.amp_var.set(str(self.amp_target))

    def _amp_plus(self):
        self._set_amp_target(self.amp_target + self.amp_step)
        self._send_cmd_char('+')  # +0000000

    def _amp_minus(self):
        self._set_amp_target(self.amp_target - self.amp_step)
        self._send_cmd_char('-')  # -0000000

    def _amp_digit(self, d: int):
        self._set_amp_target(d * 5)
        self._send_cmd_char(str(d))  # '7' -> 70000000

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

    # ---------------- Mode panel logic ----------------
    def _on_mode_change(self):
        mode = self.control_mode_var.get()
        auto = (mode == "auto")

        # D-pad enabled only in manual
        self.dpad_enabled = not auto
        self._apply_dpad_visual_state()

        # Auto widgets enabled only in auto
        state_auto = "normal" if auto else "disabled"
        self.rb_angle.config(state=state_auto)
        self.rb_coords.config(state=state_auto)

        # Send mode marker (8B head+zeros)
        if not auto:
            self._send_8b_head("M", 0)  # manual
        else:
            if self.auto_submode_var.get() == "angle":
                self._send_8b_head("D", 0)  # direction/angle
            else:
                self._send_8b_head("P", 0)  # position/coords

        self._on_auto_submode_change()

    def _on_auto_submode_change(self):
        auto = (self.control_mode_var.get() == "auto")
        sub = self.auto_submode_var.get()

        def set_widgets(widgets, st):
            for w in widgets:
                try:
                    w.config(state=st)
                except tk.TclError:
                    pass

        if not auto:
            set_widgets([self.angle_entry, self.btn_send_angle], "disabled")
            set_widgets([self.xcmd_entry, self.btn_send_x, self.ycmd_entry, self.btn_send_y, self.btn_send_xy], "disabled")
            return

        # switching submode while in auto also sends the corresponding marker
        if sub == "angle":
            self._send_8b_head("D", 0)
            set_widgets([self.angle_entry, self.btn_send_angle], "normal")
            set_widgets([self.xcmd_entry, self.btn_send_x, self.ycmd_entry, self.btn_send_y, self.btn_send_xy], "disabled")
        else:
            self._send_8b_head("P", 0)
            set_widgets([self.angle_entry, self.btn_send_angle], "disabled")
            set_widgets([self.xcmd_entry, self.btn_send_x, self.ycmd_entry, self.btn_send_y, self.btn_send_xy], "normal")

    def _send_angle(self):
        if self.control_mode_var.get() != "auto" or self.auto_submode_var.get() != "angle":
            return
        try:
            ang = int(self.angle_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid angle", "Angle must be an integer (e.g., 120).")
            return
        ang = max(0, min(359, ang))
        self.angle_var.set(str(ang))
        # Send digits only (8 bytes)
        self._send_8b_number(ang)

    def _send_x(self):
        if self.control_mode_var.get() != "auto" or self.auto_submode_var.get() != "coords":
            return
        try:
            x = int(self.xcmd_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid X", "X must be an integer (e.g., 100).")
            return
        self.xcmd_var.set(str(x))
        self._send_8b_number(x)

    def _send_y(self):
        if self.control_mode_var.get() != "auto" or self.auto_submode_var.get() != "coords":
            return
        try:
            y = int(self.ycmd_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Y", "Y must be an integer (e.g., 200).")
            return
        self.ycmd_var.set(str(y))
        self._send_8b_number(y)

    def _send_xy(self):
        self._send_x()
        time.sleep(0.02)
        self._send_y()

    # ---------------- D-pad drawing ----------------
    def _draw_arrows(self):
        arrow_r_outer = self.base_r - 10
        arrow_r_inner = self.base_r - 45
        w = 22

        def tri(points, tag):
            _id = self.canvas.create_polygon(points, fill="#f2f2f2", outline="#777", width=2, tags=(tag,))
            self.arrow_ids.append(_id)
            return _id

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

        self.canvas.tag_bind("arrow_up", "<Button-1>", lambda e: self._arrow_click("up"))
        self.canvas.tag_bind("arrow_right", "<Button-1>", lambda e: self._arrow_click("right"))
        self.canvas.tag_bind("arrow_down", "<Button-1>", lambda e: self._arrow_click("down"))
        self.canvas.tag_bind("arrow_left", "<Button-1>", lambda e: self._arrow_click("left"))

    def _apply_dpad_visual_state(self):
        if self.dpad_enabled:
            self.canvas.itemconfig(self.base_circle, outline="#999", width=3)
            for aid in self.arrow_ids:
                self.canvas.itemconfig(aid, fill="#f2f2f2", outline="#777", width=2)
            self.canvas.itemconfig(self.knob, fill="#ddd", outline="#666", width=2)
        else:
            self._set_knob_center()
            self.canvas.itemconfig(self.base_circle, outline="#c8c8c8", width=3)
            for aid in self.arrow_ids:
                self.canvas.itemconfig(aid, fill="#efefef", outline="#bdbdbd", width=2)
            self.canvas.itemconfig(self.knob, fill="#f0f0f0", outline="#bdbdbd", width=2)

    def _on_canvas_resize(self, event):
        w, h = event.width, event.height
        if w < 10 or h < 10:
            return
        margin = 20
        min_base_r = 25  # ensure base radius stays large enough for valid knob motion
        min_dim = min(w, h)
        available_base_r = min_dim // 2 - margin
        if available_base_r < min_base_r:
            # Canvas too small to render the D-pad reliably; keep previous layout
            return
        self.cx = w // 2
        self.cy = h // 2
        self.base_r = available_base_r
        self.knob_r = max(15, self.base_r // 6)
        self.deadzone_px = max(15, self.base_r // 5)

        self.canvas.coords(
            self.base_circle,
            self.cx - self.base_r, self.cy - self.base_r,
            self.cx + self.base_r, self.cy + self.base_r,
        )

        for aid in self.arrow_ids:
            self.canvas.delete(aid)
        self.arrow_ids = []
        self._draw_arrows()

        self._set_knob_center()
        self._apply_dpad_visual_state()

    def _arrow_click(self, direction: str):
        if not self.dpad_enabled:
            return

        if direction == "left":
            self._set_knob_from_normalized(-1.0, 0.0)
            self._send_manual_angle(-90, force=True)
        elif direction == "right":
            self._set_knob_from_normalized(1.0, 0.0)
            self._send_manual_angle(90, force=True)
        elif direction == "up":
            self._set_knob_from_normalized(0.0, 1.0)
            self._send_manual_angle(0, force=True)
        elif direction == "down":
            self._set_knob_from_normalized(0.0, -1.0)
            self._send_manual_angle(0, force=True)

    # ---------------- D-pad interactions ----------------
    def on_canvas_click(self, event):
        if not self.dpad_enabled:
            return
        dx, dy = event.x - self.cx, event.y - self.cy
        if math.hypot(dx, dy) <= self.base_r:
            self.dragging = True
            self._update_stick(event.x, event.y, send_now=True)

    def on_canvas_drag(self, event):
        if not self.dpad_enabled:
            return
        if not self.dragging:
            return
        self._update_stick(event.x, event.y, send_now=True)

    def on_canvas_release(self, event):
        if not self.dpad_enabled:
            return
        if not self.dragging:
            return
        self.dragging = False
        self._set_knob_center()
        self.last_dpad_cmd = None
        self._send_manual_angle(0, force=True)

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
            self._send_manual_angle(0)
            return

        # Curvature-based mapping: use stick angle on the circular pad arc.
        # This follows the circle geometry instead of a linear X mapping.
        radius = max(1.0, math.hypot(dx, dy))
        x_over_r = max(-1.0, min(1.0, dx / radius))
        angle = math.degrees(math.asin(x_over_r))
        self._send_manual_angle(angle)

    # ---------------- Close ----------------
    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    root.resizable(True, True)
    root.minsize(600, 500)
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass

    TelemetryUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()