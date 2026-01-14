import bluetooth
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

DEFAULT_ESP32_ADDR = "E0:8C:FE:5C:E2:A6"
DEFAULT_CHANNEL = 1  # RFCOMM channel

class ESP32ControllerUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Bluetooth Controller (RFCOMM)")
        self.root.geometry("520x420")

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

        # ===== Controls =====
        controls = ttk.LabelFrame(root, text="Controls")
        controls.pack(fill="x", padx=10, pady=10)

        # Big buttons row
        btn_frame = ttk.Frame(controls)
        btn_frame.pack(fill="x", padx=10, pady=10)

        self.left_btn = ttk.Button(btn_frame, text="⬅ Left (l)", command=lambda: self.send_cmd("l"), state="disabled")
        self.left_btn.grid(row=0, column=0, sticky="ew", padx=6, pady=6)

        self.straight_btn = ttk.Button(btn_frame, text="⬆ Straight (s)", command=lambda: self.send_cmd("s"), state="disabled")
        self.straight_btn.grid(row=0, column=1, sticky="ew", padx=6, pady=6)

        self.right_btn = ttk.Button(btn_frame, text="Right ➡ (r)", command=lambda: self.send_cmd("r"), state="disabled")
        self.right_btn.grid(row=0, column=2, sticky="ew", padx=6, pady=6)

        self.cal_btn = ttk.Button(btn_frame, text="Calibrate (0)", command=lambda: self.send_cmd("0"), state="disabled")
        self.cal_btn.grid(row=1, column=0, columnspan=3, sticky="ew", padx=6, pady=6)

        btn_frame.columnconfigure(0, weight=1)
        btn_frame.columnconfigure(1, weight=1)
        btn_frame.columnconfigure(2, weight=1)

        ttk.Label(
            controls,
            text="Keyboard: A=left, D=right, W/S=straight, 0=calibrate, Esc=disconnect",
        ).pack(anchor="w", padx=10, pady=(0, 10))

        # ===== Log =====
        log_frame = ttk.LabelFrame(root, text="Log")
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.log = tk.Text(log_frame, height=8, wrap="word")
        self.log.pack(fill="both", expand=True, padx=8, pady=8)
        self.log.configure(state="disabled")

        # Key binds
        root.bind("<a>", lambda e: self.send_cmd("l"))
        root.bind("<A>", lambda e: self.send_cmd("l"))
        root.bind("<d>", lambda e: self.send_cmd("r"))
        root.bind("<D>", lambda e: self.send_cmd("r"))
        root.bind("<w>", lambda e: self.send_cmd("s"))
        root.bind("<W>", lambda e: self.send_cmd("s"))
        root.bind("<s>", lambda e: self.send_cmd("s"))
        root.bind("<S>", lambda e: self.send_cmd("s"))
        root.bind("<Key-0>", lambda e: self.send_cmd("0"))
        root.bind("<Escape>", lambda e: self.disconnect())

        # Clean close
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def append_log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def set_connected_ui(self, is_connected: bool):
        self.connected = is_connected
        self.connect_btn.configure(state="disabled" if is_connected else "normal")
        self.disconnect_btn.configure(state="normal" if is_connected else "disabled")
        for b in (self.left_btn, self.right_btn, self.straight_btn, self.cal_btn):
            b.configure(state="normal" if is_connected else "disabled")

    def connect_async(self):
        if self.connected:
            return
        addr = self.addr_var.get().strip()
        channel = int(self.channel_var.get())
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

    def send_cmd(self, cmd: str):
        if not self.connected or not self.sock:
            return
        if cmd not in ("l", "r", "s", "0"):
            return

        # throttle similar to your 0.05 sleep
        def worker():
            with self.sending_lock:
                try:
                    self.sock.send(cmd)
                    self.root.after(0, lambda: self.append_log(f"Sent: {cmd}"))
                    time.sleep(0.05)
                except Exception as e:
                    self.root.after(0, lambda: self.append_log(f"Send failed: {e}"))
                    self.root.after(0, self.disconnect)

        threading.Thread(target=worker, daemon=True).start()

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ESP32ControllerUI(root)
    root.mainloop()
