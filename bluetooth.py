import bluetooth
import threading
import time
import math
import struct
import tkinter as tk
from tkinter import ttk, messagebox

DEFAULT_ESP32_ADDR = "E0:8C:FE:5C:E2:A6"
DEFAULT_CHANNEL = 1

TELEMETRY_FRAME_SIZE = 12  # 3 float32 little-endian: <fff
RECV_CHUNK = 1024


class ESP32ControllerUI:
    """
    ESP32 protocol (based on your C++ code):
      PC -> ESP32 (ASCII single-byte commands):
        c/C : calibrate + STOP
        l/L : turn left
        r/R : turn right
        s/S : straighten + RESUME
        0..9: amplitude = digit*5 degrees
        + / -: amplitude +/- 5

      ESP32 -> PC (binary telemetry, 12 bytes/frame):
        struct.pack("<fff", direction_deg, xw_m, yw_m)
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Snake Controller + Telemetry (RFCOMM)")
        self.root.geometry("740x640")

        self.sock = None
        self.connected = False
        self.sending_lock = threading.Lock()

        # Receiver thread state
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.rx_buffer = bytearray()

        # Latest telemetry (for UI)
        self.dir_var = tk.StringVar(value="—")
        self.xw_var = tk.StringVar(value="—")
        self.yw_var = tk.StringVar(value="—")
        self.last_rx_var = tk.StringVar(value="Never")

        # ===== Top: connection frame =====
        conn = ttk.LabelFrame(root, text="Connection")
        conn.pack(fill="x", padx=10, pady=10)

        ttk.Label(conn, text="ESP32 MAC:").grid(row=0, column=0, sticky="w", padx=8, pady=6)
        self.addr_var = tk.StringVar(value=DEFAULT_ESP32_ADDR)
        self.addr_entry = ttk.Entry(conn, textvariable=self.addr_var, width=24)
        self.addr_entry.grid(row=0, column=1, padx=8, pady=6)

        ttk.Label(conn, text="Channel:").grid(row=0, column=2, sticky="w", padx=8, pady=6)
        self.channel_var = tk.IntVar(value=DEFAULT_CHANNEL)
        self.channel_spin = ttk.Spinbox(conn, from_=1, to=30, textvariable=self.channel_var, width=5)
        self.channel_spin.grid(row=0, column=3, padx=8, pady=6)

        self.connect_btn = ttk.Button(conn, text="Connect", command=self.connect_async)
        self.connect_btn.grid(row=1, column=1, sticky="ew", padx=8, pady=8)

        self.disconnect_btn = ttk.Button(conn, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=1, column=2, sticky="ew", padx=8, pady=8)

        conn.columnconfigure(1, weight=1)
        conn.columnconfigure(2, weight=1)

        # ===== Status =====
        self.status_var = tk.StringVar(value="Not connected.")
        ttk.Label(root, textvariable=self.status_var).pack(anchor="w", padx=12)

        # ===== Telemetry panel =====
        telem = ttk.LabelFrame(root, text="Telemetry (from ESP32, 10 Hz, 12 bytes/frame)")
        telem.pack(fill="x", padx=10, pady=10)

        grid = ttk.Frame(telem)
        grid.pack(fill="x", padx=10, pady=10)

        ttk.Label(grid, text="Direction (deg):").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.dir_var, width=18).grid(row=0, column=1, sticky="w", padx=6, pady=4)

        ttk.Label(grid, text="Xw (m):").grid(row=0, column=2, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.xw_var, width=18).grid(row=0, column=3, sticky="w", padx=6, pady=4)

        ttk.Label(grid, text="Yw (m):").grid(row=0, column=4, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.yw_var, width=18).grid(row=0, column=5, sticky="w", padx=6, pady=4)

        ttk.Label(grid, text="Last RX:").grid(row=1, column=0, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.last_rx_var).grid(row=1, column=1, columnspan=5, sticky="w", padx=6, pady=4)

        for c in range(6):
            grid.columnconfigure(c, weight=1)

        # ===== Controls =====
        controls = ttk.LabelFrame(root, text="Controls (match ESP32 single-char commands)")
        controls.pack(fill="x", padx=10, pady=10)

        # Row 0: primary buttons
        btns = ttk.Frame(controls)
        btns.pack(fill="x", padx=10, pady=10)

        self.btn_cal = ttk.Button(btns, text="Calibrate/Stop (c)", command=lambda: self.send_cmd_char('c'),
                                  state="disabled")
        self.btn_left = ttk.Button(btns, text="Left (l)", command=lambda: self.send_cmd_char('l'),
                                   state="disabled")
        self.btn_right = ttk.Button(btns, text="Right (r)", command=lambda: self.send_cmd_char('r'),
                                    state="disabled")
        self.btn_straight = ttk.Button(btns, text="Straight/Run (s)", command=lambda: self.send_cmd_char('s'),
                                       state="disabled")

        self.btn_cal.pack(side="left", padx=6)
        self.btn_left.pack(side="left", padx=6)
        self.btn_right.pack(side="left", padx=6)
        self.btn_straight.pack(side="left", padx=6)

        # Row 1: amplitude controls
        amp = ttk.Frame(controls)
        amp.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(amp, text="Amplitude:").pack(side="left", padx=(0, 8))

        self.amp_var = tk.IntVar(value=6)  # digit 0..9 => amp = digit*5
        self.amp_spin = ttk.Spinbox(amp, from_=0, to=9, textvariable=self.amp_var, width=5, state="disabled")
        self.amp_spin.pack(side="left", padx=6)

        self.btn_set_amp = ttk.Button(amp, text="Send digit (0..9)", command=self.send_amp_digit, state="disabled")
        self.btn_minus = ttk.Button(amp, text="- (decrease)", command=lambda: self.send_cmd_char('-'), state="disabled")
        self.btn_plus = ttk.Button(amp, text="+ (increase)", command=lambda: self.send_cmd_char('+'), state="disabled")

        self.btn_set_amp.pack(side="left", padx=6)
        self.btn_minus.pack(side="left", padx=6)
        self.btn_plus.pack(side="left", padx=6)

        # ===== Circular D-pad =====
        dpad_frame = ttk.LabelFrame(root, text="D-pad (sends l/r/s; click center sends s)")
        dpad_frame.pack(fill="x", padx=10, pady=10)

        ttk.Label(
            dpad_frame,
            text="Up/Down => Straight/Run (s)\nLeft => l, Right => r, Center => s",
        ).pack(anchor="w", padx=10, pady=(6, 0))

        self.canvas = tk.Canvas(dpad_frame, width=320, height=320, highlightthickness=0)
        self.canvas.pack(padx=10, pady=10)

        self.cx, self.cy = 160, 160
        self.base_r = 130
        self.knob_r = 22

        self.base_circle = self.canvas.create_oval(
            self.cx - self.base_r, self.cy - self.base_r,
            self.cx + self.base_r, self.cy + self.base_r,
            outline="#999", width=3
        )

        self.arrow_items = {}
        self._draw_arrows()

        self.knob = self.canvas.create_oval(
            self.cx - self.knob_r, self.cy - self.knob_r,
            self.cx + self.knob_r, self.cy + self.knob_r,
            fill="#ddd", outline="#666", width=2
        )

        self.dragging = False
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

        # ===== Log =====
        log_frame = ttk.LabelFrame(root, text="Log")
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.log = tk.Text(log_frame, height=10, wrap="word")
        self.log.pack(fill="both", expand=True, padx=8, pady=8)
        self.log.configure(state="disabled")

        # Key binds
        root.bind("<Escape>", lambda e: self.disconnect())
        root.bind("<Key-c>", lambda e: self.send_cmd_char('c'))
        root.bind("<Key-l>", lambda e: self.send_cmd_char('l'))
        root.bind("<Key-r>", lambda e: self.send_cmd_char('r'))
        root.bind("<Key-s>", lambda e: self.send_cmd_char('s'))
        root.bind("<Key-plus>", lambda e: self.send_cmd_char('+'))
        root.bind("<Key-minus>", lambda e: self.send_cmd_char('-'))

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- UI drawing ----------------

    def _draw_arrows(self):
        arrow_r_outer = self.base_r - 10
        arrow_r_inner = self.base_r - 45
        w = 22

        def tri(points, tag):
            return self.canvas.create_polygon(points, fill="#f2f2f2", outline="#777", width=2, tags=(tag,))

        up_tip = (self.cx, self.cy - arrow_r_outer)
        up_left = (self.cx - w, self.cy - arrow_r_inner)
        up_right = (self.cx + w, self.cy - arrow_r_inner)
        tri([up_tip, up_left, up_right], "arrow_up")

        rt_tip = (self.cx + arrow_r_outer, self.cy)
        rt_up = (self.cx + arrow_r_inner, self.cy - w)
        rt_dn = (self.cx + arrow_r_inner, self.cy + w)
        tri([rt_tip, rt_up, rt_dn], "arrow_right")

        dn_tip = (self.cx, self.cy + arrow_r_outer)
        dn_left = (self.cx - w, self.cy + arrow_r_inner)
        dn_right = (self.cx + w, self.cy + arrow_r_inner)
        tri([dn_tip, dn_left, dn_right], "arrow_down")

        lf_tip = (self.cx - arrow_r_outer, self.cy)
        lf_up = (self.cx - arrow_r_inner, self.cy - w)
        lf_dn = (self.cx - arrow_r_inner, self.cy + w)
        tri([lf_tip, lf_up, lf_dn], "arrow_left")

        self.canvas.tag_bind("arrow_up", "<Button-1>", lambda e: self.on_arrow("up"))
        self.canvas.tag_bind("arrow_right", "<Button-1>", lambda e: self.on_arrow("right"))
        self.canvas.tag_bind("arrow_down", "<Button-1>", lambda e: self.on_arrow("down"))
        self.canvas.tag_bind("arrow_left", "<Button-1>", lambda e: self.on_arrow("left"))

    # ---------------- Logging ----------------

    def append_log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def set_connected_ui(self, is_connected: bool):
        self.connected = is_connected
        self.connect_btn.configure(state="disabled" if is_connected else "normal")
        self.disconnect_btn.configure(state="normal" if is_connected else "disabled")

        # enable controls
        state = "normal" if is_connected else "disabled"
        self.btn_cal.configure(state=state)
        self.btn_left.configure(state=state)
        self.btn_right.configure(state=state)
        self.btn_straight.configure(state=state)
        self.amp_spin.configure(state=state)
        self.btn_set_amp.configure(state=state)
        self.btn_minus.configure(state=state)
        self.btn_plus.configure(state=state)

    # ---------------- Bluetooth connect/disconnect ----------------

    def connect_async(self):
        if self.connected:
            return

        addr = self.addr_var.get().strip()
        try:
            channel = int(self.channel_var.get())
        except Exception:
            messagebox.showerror("Error", "Channel must be an integer.")
            return

        if not addr:
            messagebox.showerror("Error", "Please enter an ESP32 MAC address.")
            return

        self.status_var.set(f"Connecting to {addr} (ch {channel})...")
        self.append_log(f"Trying to connect to {addr} on channel {channel}...")
        self.connect_btn.configure(state="disabled")

        def worker():
            try:
                sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                sock.connect((addr, channel))
                # optional: lower latency; safe to ignore if unsupported
                try:
                    sock.settimeout(1.0)
                except Exception:
                    pass
                self.sock = sock
                self.root.after(0, lambda: self.on_connected(addr, channel))
            except Exception as e:
                self.root.after(0, lambda err=e: self.on_connect_failed(err))

        threading.Thread(target=worker, daemon=True).start()

    def on_connected(self, addr, channel):
        self.set_connected_ui(True)
        self.status_var.set(f"Connected to {addr} (ch {channel}).")
        self.append_log("Connected!")

        # Start receiver thread
        self.rx_stop.clear()
        self.rx_buffer = bytearray()
        self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        self.rx_thread.start()
        self.append_log("RX thread started (telemetry).")

    def on_connect_failed(self, e: Exception):
        self.sock = None
        self.set_connected_ui(False)
        self.status_var.set("Not connected.")
        self.append_log(f"Failed to connect: {e}")
        messagebox.showerror("Connection failed", str(e))

    def disconnect(self):
        if not self.connected:
            return

        self.append_log("Disconnecting...")
        self.rx_stop.set()

        try:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
        finally:
            self.sock = None
            self.set_connected_ui(False)
            self.status_var.set("Not connected.")
            self.append_log("Disconnected.")

    # ---------------- Sending (single-char commands) ----------------

    def send_cmd_char(self, ch: str):
        """Send exactly one command byte to match ESP32 processCommand()."""
        if not self.connected or not self.sock:
            return
        if not ch or len(ch) != 1:
            return

        def worker():
            with self.sending_lock:
                try:
                    self.sock.send(ch.encode("ascii", errors="ignore"))
                    self.root.after(0, lambda: self.append_log(f"Sent cmd: {repr(ch)}"))
                    time.sleep(0.01)
                except Exception as e:
                    self.root.after(0, lambda: self.append_log(f"Send failed: {e}"))
                    self.root.after(0, self.disconnect)

        threading.Thread(target=worker, daemon=True).start()

    def send_amp_digit(self):
        d = int(self.amp_var.get())
        d = max(0, min(9, d))
        self.send_cmd_char(chr(ord('0') + d))

    # ---------------- Receiving telemetry ----------------

    def _rx_worker(self):
        """
        Continuously read bytes from RFCOMM and parse fixed 12-byte frames.
        Frames might arrive split or coalesced, so we buffer.
        """
        while not self.rx_stop.is_set():
            if not self.sock:
                break
            try:
                chunk = self.sock.recv(RECV_CHUNK)
                if not chunk:
                    # peer closed
                    self.root.after(0, lambda: self.append_log("RX: connection closed by peer."))
                    self.root.after(0, self.disconnect)
                    return

                self.rx_buffer.extend(chunk)

                # Parse as many frames as possible
                while len(self.rx_buffer) >= TELEMETRY_FRAME_SIZE:
                    frame = bytes(self.rx_buffer[:TELEMETRY_FRAME_SIZE])
                    del self.rx_buffer[:TELEMETRY_FRAME_SIZE]

                    try:
                        direction_deg, xw_m, yw_m = struct.unpack("<fff", frame)
                    except Exception as e:
                        # If decoding fails, drop buffer to resync (rare)
                        self.rx_buffer.clear()
                        self.root.after(0, lambda: self.append_log(f"Telemetry unpack error: {e}"))
                        break

                    # Push UI update safely
                    self.root.after(
                        0,
                        lambda d=direction_deg, x=xw_m, y=yw_m: self._update_telem_ui(d, x, y)
                    )

            except bluetooth.btcommon.BluetoothError:
                # timeouts are fine; keep looping
                continue
            except Exception as e:
                self.root.after(0, lambda: self.append_log(f"RX error: {e}"))
                self.root.after(0, self.disconnect)
                return

    def _update_telem_ui(self, direction_deg: float, xw_m: float, yw_m: float):
        self.dir_var.set(f"{direction_deg:.2f}")
        self.xw_var.set(f"{xw_m:.3f}")
        self.yw_var.set(f"{yw_m:.3f}")
        self.last_rx_var.set(time.strftime("%H:%M:%S"))

    # ---------------- D-pad behavior ----------------

    def on_arrow(self, direction: str):
        if direction == "left":
            self.send_cmd_char('l')
            self._set_knob_from_normalized(-1.0, 0.0)
        elif direction == "right":
            self.send_cmd_char('r')
            self._set_knob_from_normalized(1.0, 0.0)
        elif direction == "up":
            self.send_cmd_char('s')
            self._set_knob_from_normalized(0.0, 1.0)
        elif direction == "down":
            self.send_cmd_char('s')
            self._set_knob_from_normalized(0.0, -1.0)

    def on_canvas_click(self, event):
        dx, dy = event.x - self.cx, event.y - self.cy
        dist = math.hypot(dx, dy)
        if dist <= self.base_r:
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
        # On release, straighten & keep running
        self.send_cmd_char('s')

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

        # Decide command based on stick angle (simple mapping)
        # left/right => l/r ; otherwise => s
        if not send_now:
            return

        # Deadzone near center -> straight
        if math.hypot(dx, dy) < 20:
            self.send_cmd_char('s')
            return

        ang = (math.degrees(math.atan2(-dy, dx)) + 360.0) % 360.0  # screen->math
        if 135 <= ang <= 225:
            self.send_cmd_char('l')
        elif ang <= 45 or ang >= 315:
            self.send_cmd_char('r')
        else:
            self.send_cmd_char('s')

    # ---------------- Close ----------------

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ESP32ControllerUI(root)
    root.mainloop()
