import bluetooth
import threading
import time
import math
import tkinter as tk
from tkinter import ttk, messagebox

DEFAULT_ESP32_ADDR = "E0:8C:FE:5C:E2:A6"
DEFAULT_CHANNEL = 1


class ESP32ControllerUI:
    """
    Two send modes:
      1) Angle mode: sends an integer angle in degrees (e.g., "90\n")
      2) Coordinate mode: sends a pair of floats "x,y\n" (e.g., "0.25,-0.80\n")

    Notes:
      - Everything is sent as ASCII text with a trailing newline.
      - If your ESP32 code currently expects single chars ('l','r','s','0'), you must update it
        to parse these payloads.
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Bluetooth Controller (RFCOMM)")
        self.root.geometry("620x560")

        self.sock = None
        self.connected = False
        self.sending_lock = threading.Lock()

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

        # ===== Modes + Controls container =====
        top_controls = ttk.Frame(root)
        top_controls.pack(fill="x", padx=10, pady=10)

        mode_frame = ttk.LabelFrame(top_controls, text="Send Mode")
        mode_frame.pack(side="left", fill="y", padx=(0, 10))

        self.mode_var = tk.StringVar(value="angle")  # "angle" or "coords"
        ttk.Radiobutton(mode_frame, text="Angle (int degrees)", value="angle", variable=self.mode_var,
                        command=self.on_mode_change).pack(anchor="w", padx=10, pady=(8, 2))
        ttk.Radiobutton(mode_frame, text="Coordinates (x,y floats)", value="coords", variable=self.mode_var,
                        command=self.on_mode_change).pack(anchor="w", padx=10, pady=(2, 8))

        payload_frame = ttk.LabelFrame(top_controls, text="Payload")
        payload_frame.pack(side="left", fill="both", expand=True)

        # Angle payload widgets
        self.angle_var = tk.IntVar(value=90)
        self.angle_label = ttk.Label(payload_frame, text="Angle (deg):")
        self.angle_spin = ttk.Spinbox(payload_frame, from_=0, to=359, textvariable=self.angle_var, width=8)
        self.angle_send_btn = ttk.Button(payload_frame, text="Send Angle", command=self.send_angle, state="disabled")

        # Coords payload widgets
        self.x_var = tk.DoubleVar(value=0.0)
        self.y_var = tk.DoubleVar(value=0.0)
        self.coords_label = ttk.Label(payload_frame, text="Coords (x,y):")
        self.x_entry = ttk.Entry(payload_frame, textvariable=self.x_var, width=10)
        self.y_entry = ttk.Entry(payload_frame, textvariable=self.y_var, width=10)
        self.coords_send_btn = ttk.Button(payload_frame, text="Send Coords", command=self.send_coords, state="disabled")

        # Calibrate always available when connected
        self.cal_btn = ttk.Button(payload_frame, text="Calibrate", command=self.calibrate, state="disabled")

        # Layout inside payload_frame (grid so we can show/hide rows)
        payload_frame.columnconfigure(3, weight=1)

        self.angle_label.grid(row=0, column=0, sticky="w", padx=10, pady=(10, 6))
        self.angle_spin.grid(row=0, column=1, sticky="w", padx=6, pady=(10, 6))
        self.angle_send_btn.grid(row=0, column=2, sticky="w", padx=6, pady=(10, 6))

        self.coords_label.grid(row=1, column=0, sticky="w", padx=10, pady=6)
        ttk.Label(payload_frame, text="x:").grid(row=1, column=1, sticky="e", padx=(6, 0), pady=6)
        self.x_entry.grid(row=1, column=2, sticky="w", padx=(6, 12), pady=6)
        ttk.Label(payload_frame, text="y:").grid(row=1, column=3, sticky="e", padx=(6, 0), pady=6)
        self.y_entry.grid(row=1, column=4, sticky="w", padx=(6, 12), pady=6)
        self.coords_send_btn.grid(row=1, column=5, sticky="w", padx=6, pady=6)

        self.cal_btn.grid(row=2, column=0, columnspan=6, sticky="ew", padx=10, pady=(6, 10))

        # ===== Circular D-pad (Xbox-ish) =====
        dpad_frame = ttk.LabelFrame(root, text="D-pad (circular)")
        dpad_frame.pack(fill="x", padx=10, pady=10)

        ttk.Label(
            dpad_frame,
            text="Click arrows or drag the stick. Mode decides what gets sent.\n"
                 "Angle mode sends angle degrees; Coords mode sends normalized x,y in [-1,1].",
        ).pack(anchor="w", padx=10, pady=(6, 0))

        self.canvas = tk.Canvas(dpad_frame, width=320, height=320, highlightthickness=0)
        self.canvas.pack(padx=10, pady=10)

        # Geometry
        self.cx, self.cy = 160, 160
        self.base_r = 130
        self.knob_r = 22
        self.deadzone = 0.08  # coords deadzone radius (normalized)

        # Draw base circle
        self.base_circle = self.canvas.create_oval(
            self.cx - self.base_r, self.cy - self.base_r,
            self.cx + self.base_r, self.cy + self.base_r,
            outline="#999", width=3
        )

        # Draw arrows as wedge-ish triangles at 4 cardinal directions
        self.arrow_items = {}
        self._draw_arrows()

        # Draw knob
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

        # Key binds (optional, keep calibrate + escape)
        root.bind("<Key-0>", lambda e: self.calibrate())
        root.bind("<Escape>", lambda e: self.disconnect())

        # Clean close
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Initial mode visibility
        self.on_mode_change()

    # ---------------- UI drawing ----------------

    def _draw_arrows(self):
        # Simple triangles; clicking them triggers direction.
        # Angles: up=90, right=0, down=270, left=180 (math-y; but you can change mapping below).
        # We'll use conventional compass-ish: up=90, right=0, down=270, left=180.
        arrow_r_outer = self.base_r - 10
        arrow_r_inner = self.base_r - 45
        w = 22  # half-width of arrow base

        def tri(points, tag):
            item = self.canvas.create_polygon(points, fill="#f2f2f2", outline="#777", width=2, tags=(tag,))
            return item

        # Up
        up_tip = (self.cx, self.cy - arrow_r_outer)
        up_left = (self.cx - w, self.cy - arrow_r_inner)
        up_right = (self.cx + w, self.cy - arrow_r_inner)
        self.arrow_items["up"] = tri([up_tip, up_left, up_right], "arrow_up")

        # Right
        rt_tip = (self.cx + arrow_r_outer, self.cy)
        rt_up = (self.cx + arrow_r_inner, self.cy - w)
        rt_dn = (self.cx + arrow_r_inner, self.cy + w)
        self.arrow_items["right"] = tri([rt_tip, rt_up, rt_dn], "arrow_right")

        # Down
        dn_tip = (self.cx, self.cy + arrow_r_outer)
        dn_left = (self.cx - w, self.cy + arrow_r_inner)
        dn_right = (self.cx + w, self.cy + arrow_r_inner)
        self.arrow_items["down"] = tri([dn_tip, dn_left, dn_right], "arrow_down")

        # Left
        lf_tip = (self.cx - arrow_r_outer, self.cy)
        lf_up = (self.cx - arrow_r_inner, self.cy - w)
        lf_dn = (self.cx - arrow_r_inner, self.cy + w)
        self.arrow_items["left"] = tri([lf_tip, lf_up, lf_dn], "arrow_left")

        # Bind click per arrow
        self.canvas.tag_bind("arrow_up", "<Button-1>", lambda e: self.on_arrow("up"))
        self.canvas.tag_bind("arrow_right", "<Button-1>", lambda e: self.on_arrow("right"))
        self.canvas.tag_bind("arrow_down", "<Button-1>", lambda e: self.on_arrow("down"))
        self.canvas.tag_bind("arrow_left", "<Button-1>", lambda e: self.on_arrow("left"))

    # ---------------- Logging + state ----------------

    def append_log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def set_connected_ui(self, is_connected: bool):
        self.connected = is_connected
        self.connect_btn.configure(state="disabled" if is_connected else "normal")
        self.disconnect_btn.configure(state="normal" if is_connected else "disabled")

        # Enable/disable payload send buttons based on mode + connection
        self.cal_btn.configure(state="normal" if is_connected else "disabled")
        self.angle_send_btn.configure(state=("normal" if is_connected and self.mode_var.get() == "angle" else "disabled"))
        self.coords_send_btn.configure(state=("normal" if is_connected and self.mode_var.get() == "coords" else "disabled"))

    def on_mode_change(self):
        mode = self.mode_var.get()
        # Show both rows but you could hide; easiest is disable relevant button and keep fields visible.
        if self.connected:
            self.angle_send_btn.configure(state=("normal" if mode == "angle" else "disabled"))
            self.coords_send_btn.configure(state=("normal" if mode == "coords" else "disabled"))

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
                self.sock = sock
                self.root.after(0, lambda: self.on_connected(addr, channel))
            except Exception as e:
                self.root.after(0, lambda err=e: self.on_connect_failed(err))

        threading.Thread(target=worker, daemon=True).start()

    def on_connected(self, addr, channel):
        self.set_connected_ui(True)
        self.status_var.set(f"Connected to {addr} (ch {channel}).")
        self.append_log("Connected!")

    def on_connect_failed(self, e: Exception):
        self.sock = None
        self.set_connected_ui(False)
        self.status_var.set("Not connected.")
        self.append_log(f"Failed to connect: {e}")
        messagebox.showerror("Connection failed", str(e))

    def disconnect(self):
        if not self.connected:
            return
        try:
            if self.sock:
                self.append_log("Disconnecting...")
                self.sock.close()
        except Exception as e:
            self.append_log(f"Error while disconnecting: {e}")
        finally:
            self.sock = None
            self.set_connected_ui(False)
            self.status_var.set("Not connected.")
            self.append_log("Disconnected.")

    # ---------------- Sending helpers ----------------

    def _send_text_payload(self, text: str):
        """Send ASCII text with newline, from a background thread."""
        if not self.connected or not self.sock:
            return

        def worker():
            with self.sending_lock:
                try:
                    # Ensure newline-delimited messages for easy parsing on ESP32 side
                    payload = (text.strip() + "\n").encode("utf-8")
                    self.sock.send(payload)
                    self.root.after(0, lambda: self.append_log(f"Sent: {text.strip()}"))
                    time.sleep(0.03)
                except Exception as e:
                    self.root.after(0, lambda: self.append_log(f"Send failed: {e}"))
                    self.root.after(0, self.disconnect)

        threading.Thread(target=worker, daemon=True).start()

    def send_angle(self):
        # Send integer angle in degrees
        ang = int(self.angle_var.get()) % 360
        self._send_text_payload(f"A:{ang}")

    def send_coords(self):
        # Send two floats x,y
        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
        except Exception:
            messagebox.showerror("Error", "Coordinates must be numbers.")
            return
        self._send_text_payload(f"XY:{x:.4f},{y:.4f}")

    def calibrate(self):
        # Keep a simple calibrate command
        self._send_text_payload("CAL")

    # ---------------- D-pad behavior ----------------

    def on_arrow(self, direction: str):
        """
        Arrow click sends:
          - Angle mode: sets angle_var and sends it
          - Coords mode: sets (x,y) to unit vector and sends it
        """
        if direction == "up":
            ang = 90
            x, y = 0.0, 1.0
        elif direction == "right":
            ang = 0
            x, y = 1.0, 0.0
        elif direction == "down":
            ang = 270
            x, y = 0.0, -1.0
        elif direction == "left":
            ang = 180
            x, y = -1.0, 0.0
        else:
            return

        if self.mode_var.get() == "angle":
            self.angle_var.set(ang)
            self.send_angle()
        else:
            self.x_var.set(x)
            self.y_var.set(y)
            self.send_coords()

        # Visual nudge knob toward that direction (optional)
        self._set_knob_from_normalized(x, y)

    def on_canvas_click(self, event):
        # If user clicks close to knob or inside base, start dragging + update immediately
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
        # Snap back to center
        self._set_knob_center()
        # Set coords to 0,0 on release (optional)
        self.x_var.set(0.0)
        self.y_var.set(0.0)
        if self.mode_var.get() == "coords":
            self.send_coords()

    def _set_knob_center(self):
        self.canvas.coords(
            self.knob,
            self.cx - self.knob_r, self.cy - self.knob_r,
            self.cx + self.knob_r, self.cy + self.knob_r
        )

    def _set_knob_from_normalized(self, x_norm: float, y_norm: float):
        # Clamp and map [-1,1] to knob displacement within base radius - knob radius
        max_d = self.base_r - self.knob_r - 6
        x_norm = max(-1.0, min(1.0, x_norm))
        y_norm = max(-1.0, min(1.0, y_norm))
        kx = self.cx + x_norm * max_d
        ky = self.cy - y_norm * max_d  # invert for screen coords
        self.canvas.coords(
            self.knob,
            kx - self.knob_r, ky - self.knob_r,
            kx + self.knob_r, ky + self.knob_r
        )

    def _update_stick(self, x, y, send_now: bool):
        # Clamp drag point to base circle
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

        # Convert to normalized coords [-1,1]
        x_norm = dx / max_d
        y_norm = -dy / max_d  # screen->math (up positive)

        # Apply deadzone
        if math.hypot(x_norm, y_norm) < self.deadzone:
            x_norm, y_norm = 0.0, 0.0

        # Update UI vars
        self.x_var.set(float(f"{x_norm:.4f}"))
        self.y_var.set(float(f"{y_norm:.4f}"))

        # Compute angle
        if x_norm == 0.0 and y_norm == 0.0:
            # keep previous angle
            pass
        else:
            ang = int(round((math.degrees(math.atan2(y_norm, x_norm)) + 360.0) % 360.0))
            self.angle_var.set(ang)

        # Send based on mode
        if send_now:
            if self.mode_var.get() == "angle":
                self.send_angle()
            else:
                self.send_coords()

    # ---------------- Close ----------------

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ESP32ControllerUI(root)
    root.mainloop()
