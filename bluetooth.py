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
    ESP32 protocol (from your ESP32 code):

      PC -> ESP32 (single-char commands, ASCII bytes):
        l/L : turn left
        r/R : turn right
        s/S : straighten + RESUME motion
        c/C : calibrate + STOP (your "halt horse")
        0..9, +, - exist on ESP32 but this UI intentionally does NOT send them.

      ESP32 -> PC (binary telemetry, fixed 12 bytes/frame):
        struct.pack("<fff", direction_deg, xw_m, yw_m)
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Snake D-pad + Telemetry (RFCOMM)")
        self.root.geometry("700x640")

        self.sock = None
        self.connected = False
        self.sending_lock = threading.Lock()

        # RX thread state
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.rx_buffer = bytearray()

        # D-pad send throttling (avoid spamming)
        self.last_dpad_cmd = None
        self.last_cmd_time = 0.0
        self.hold_resend_sec = 0.15  # allow resend while holding direction

        # ===== Connection UI =====
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

        self.status_var = tk.StringVar(value="Not connected.")
        ttk.Label(root, textvariable=self.status_var).pack(anchor="w", padx=12)

        # ===== Telemetry Panel =====
        self.dir_var = tk.StringVar(value="—")
        self.xw_var = tk.StringVar(value="—")
        self.yw_var = tk.StringVar(value="—")
        self.last_rx_var = tk.StringVar(value="Never")

        telem = ttk.LabelFrame(root, text="Telemetry (ESP32 → PC, 12 bytes/frame, ~10 Hz)")
        telem.pack(fill="x", padx=10, pady=10)

        grid = ttk.Frame(telem)
        grid.pack(fill="x", padx=10, pady=10)

        ttk.Label(grid, text="Direction (deg):").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.dir_var, width=14).grid(row=0, column=1, sticky="w", padx=6, pady=4)

        ttk.Label(grid, text="Xw (m):").grid(row=0, column=2, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.xw_var, width=14).grid(row=0, column=3, sticky="w", padx=6, pady=4)

        ttk.Label(grid, text="Yw (m):").grid(row=0, column=4, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.yw_var, width=14).grid(row=0, column=5, sticky="w", padx=6, pady=4)

        ttk.Label(grid, text="Last RX:").grid(row=1, column=0, sticky="w", padx=6, pady=4)
        ttk.Label(grid, textvariable=self.last_rx_var).grid(row=1, column=1, columnspan=5, sticky="w", padx=6, pady=4)

        for c in range(6):
            grid.columnconfigure(c, weight=1)

        # ===== D-pad Only =====
        dpad_frame = ttk.LabelFrame(root, text="D-pad (ONLY control)")
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
        self.deadzone_px = 25  # "middle does nothing"

        # Base circle
        self.base_circle = self.canvas.create_oval(
            self.cx - self.base_r, self.cy - self.base_r,
            self.cx + self.base_r, self.cy + self.base_r,
            outline="#999", width=3
        )

        # Arrows (optional, clickable)
        self._draw_arrows()

        # Knob
        self.knob = self.canvas.create_oval(
            self.cx - self.knob_r, self.cy - self.knob_r,
            self.cx + self.knob_r, self.cy + self.knob_r,
            fill="#ddd", outline="#666", width=2
        )

        # Drag handling
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

        # Keys: only disconnect
        root.bind("<Escape>", lambda e: self.disconnect())

        # Clean close
        root.protocol("WM_DELETE_WINDOW", self.on_close)

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

        # reset D-pad command memory on connect/disconnect
        if not is_connected:
            self.last_dpad_cmd = None
            self.last_cmd_time = 0.0

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
                try:
                    sock.settimeout(1.0)  # avoid blocking forever in recv
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

        # Start RX telemetry thread
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

    # ---------------- Sending single-char commands ----------------

    def _send_cmd_char(self, ch: str):
        """
        Send exactly one command byte.
        Throttles repeated same command while holding direction.
        """
        if not self.connected or not self.sock:
            return
        if not ch or len(ch) != 1:
            return

        now = time.time()
        if ch == self.last_dpad_cmd and (now - self.last_cmd_time) < self.hold_resend_sec:
            return

        self.last_dpad_cmd = ch
        self.last_cmd_time = now

        def worker():
            with self.sending_lock:
                try:
                    self.sock.send(ch.encode("ascii"))
                    self.root.after(0, lambda: self.append_log(f"Sent cmd: {ch}"))
                except Exception as e:
                    self.root.after(0, lambda: self.append_log(f"Send failed: {e}"))
                    self.root.after(0, self.disconnect)

        threading.Thread(target=worker, daemon=True).start()

    # ---------------- Receiving telemetry ----------------

    def _rx_worker(self):
        while not self.rx_stop.is_set():
            if not self.sock:
                break
            try:
                chunk = self.sock.recv(RECV_CHUNK)
                if not chunk:
                    self.root.after(0, lambda: self.append_log("RX: connection closed by peer."))
                    self.root.after(0, self.disconnect)
                    return

                self.rx_buffer.extend(chunk)

                while len(self.rx_buffer) >= TELEMETRY_FRAME_SIZE:
                    frame = bytes(self.rx_buffer[:TELEMETRY_FRAME_SIZE])
                    del self.rx_buffer[:TELEMETRY_FRAME_SIZE]

                    try:
                        direction_deg, xw_m, yw_m = struct.unpack("<fff", frame)
                    except Exception as e:
                        self.rx_buffer.clear()  # resync
                        self.root.after(0, lambda: self.append_log(f"Telemetry unpack error: {e}"))
                        break

                    self.root.after(0, lambda d=direction_deg, x=xw_m, y=yw_m: self._update_telem_ui(d, x, y))

            except bluetooth.btcommon.BluetoothError:
                # timeout / transient
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

    # ---------------- D-pad drawing ----------------

    def _draw_arrows(self):
        arrow_r_outer = self.base_r - 10
        arrow_r_inner = self.base_r - 45
        w = 22

        def tri(points, tag):
            return self.canvas.create_polygon(points, fill="#f2f2f2", outline="#777", width=2, tags=(tag,))

        # Up
        up_tip = (self.cx, self.cy - arrow_r_outer)
        up_left = (self.cx - w, self.cy - arrow_r_inner)
        up_right = (self.cx + w, self.cy - arrow_r_inner)
        tri([up_tip, up_left, up_right], "arrow_up")

        # Right
        rt_tip = (self.cx + arrow_r_outer, self.cy)
        rt_up = (self.cx + arrow_r_inner, self.cy - w)
        rt_dn = (self.cx + arrow_r_inner, self.cy + w)
        tri([rt_tip, rt_up, rt_dn], "arrow_right")

        # Down
        dn_tip = (self.cx, self.cy + arrow_r_outer)
        dn_left = (self.cx - w, self.cy + arrow_r_inner)
        dn_right = (self.cx + w, self.cy + arrow_r_inner)
        tri([dn_tip, dn_left, dn_right], "arrow_down")

        # Left
        lf_tip = (self.cx - arrow_r_outer, self.cy)
        lf_up = (self.cx - arrow_r_inner, self.cy - w)
        lf_dn = (self.cx - arrow_r_inner, self.cy + w)
        tri([lf_tip, lf_up, lf_dn], "arrow_left")

        # Clicking arrows triggers same mapping
        self.canvas.tag_bind("arrow_up", "<Button-1>", lambda e: self._arrow_click("up"))
        self.canvas.tag_bind("arrow_right", "<Button-1>", lambda e: self._arrow_click("right"))
        self.canvas.tag_bind("arrow_down", "<Button-1>", lambda e: self._arrow_click("down"))
        self.canvas.tag_bind("arrow_left", "<Button-1>", lambda e: self._arrow_click("left"))

    def _arrow_click(self, direction: str):
        if direction == "left":
            self._send_cmd_char('l')
            self._set_knob_from_normalized(-1.0, 0.0)
        elif direction == "right":
            self._send_cmd_char('r')
            self._set_knob_from_normalized(1.0, 0.0)
        elif direction == "up":
            self._send_cmd_char('s')
            self._set_knob_from_normalized(0.0, 1.0)
        elif direction == "down":
            self._send_cmd_char('c')
            self._set_knob_from_normalized(0.0, -1.0)

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
        # middle does nothing => send nothing
        # reset last command so next drag re-sends immediately
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
        # Clamp drag point to the base circle
        dx = x - self.cx
        dy = y - self.cy
        dist = math.hypot(dx, dy)

        max_d = self.base_r - self.knob_r - 6
        if dist > max_d and dist > 1e-6:
            scale = max_d / dist
            dx *= scale
            dy *= scale

        # Move knob
        kx = self.cx + dx
        ky = self.cy + dy
        self.canvas.coords(
            self.knob,
            kx - self.knob_r, ky - self.knob_r,
            kx + self.knob_r, ky + self.knob_r
        )

        if not send_now:
            return

        # Middle does nothing (deadzone)
        if math.hypot(dx, dy) < self.deadzone_px:
            return

        # Decide command by dominant axis to keep it stable on diagonals
        # Screen coords: dy > 0 means dragging DOWN.
        if abs(dx) > abs(dy):
            # Horizontal dominates
            cmd = 'l' if dx < 0 else 'r'
        else:
            # Vertical dominates
            cmd = 's' if dy < 0 else 'c'  # UP => straight/run, DOWN => stop

        self._send_cmd_char(cmd)

    # ---------------- Close ----------------

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ESP32ControllerUI(root)
    root.mainloop()
