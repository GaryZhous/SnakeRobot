# import serial

# PORT = "COM10"       # <-- change this
# BAUD = 115200

# ser = serial.Serial(PORT, BAUD, timeout=1)
# print("Connected. Type a letter and press Enter (q to quit).")

# while True:
#     ch = input("> ")
#     if ch.lower() == "q":
#         break
#     if len(ch) == 0:
#         continue

#     ser.write(ch[0].encode())   # send first character
#     print(f"Sent: {ch[0]}")

# ser.close()
# print("Disconnected.")


import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports


DEFAULT_COM_PORT = "COM10"
DEFAULT_BAUD = 115200


class ESP32ControllerUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Bluetooth Controller (Serial over BT / SPP)")
        self.root.geometry("560x460")

        self.ser = None
        self.connected = False
        self.sending_lock = threading.Lock()

        # ===== Top: connection frame =====
        conn = ttk.LabelFrame(root, text="Connection")
        conn.pack(fill="x", padx=10, pady=10)

        ttk.Label(conn, text="COM Port:").grid(row=0, column=0, sticky="w", padx=8, pady=6)
        self.port_var = tk.StringVar(value=DEFAULT_COM_PORT)
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=8, pady=6, sticky="w")

        self.refresh_btn = ttk.Button(conn, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=2, padx=8, pady=6, sticky="w")

        ttk.Label(conn, text="Baud:").grid(row=0, column=3, sticky="w", padx=8, pady=6)
        self.baud_var = tk.IntVar(value=DEFAULT_BAUD)
        self.baud_entry = ttk.Entry(conn, textvariable=self.baud_var, width=10)
        self.baud_entry.grid(row=0, column=4, padx=8, pady=6, sticky="w")

        self.connect_btn = ttk.Button(conn, text="Connect", command=self.connect_async)
        self.connect_btn.grid(row=1, column=1, sticky="ew", padx=8, pady=8)

        self.disconnect_btn = ttk.Button(conn, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=1, column=2, sticky="ew", padx=8, pady=8)

        conn.columnconfigure(1, weight=1)
        conn.columnconfigure(2, weight=0)

        # ===== Status =====
        self.status_var = tk.StringVar(value="Not connected.")
        ttk.Label(root, textvariable=self.status_var).pack(anchor="w", padx=12)

        # ===== Controls =====
        controls = ttk.LabelFrame(root, text="Controls")
        controls.pack(fill="x", padx=10, pady=10)

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

        self.log = tk.Text(log_frame, height=10, wrap="word")
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

        # Fill COM dropdown on launch
        self.refresh_ports()

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        port_names = [p.device for p in ports]

        # Try to bring Bluetooth ones to the top (description varies)
        def score(p):
            desc = (p.description or "").lower()
            return 0 if "bluetooth" in desc else 1

        ports_sorted = sorted(ports, key=score)
        port_names_sorted = [p.device for p in ports_sorted]

        self.port_combo["values"] = port_names_sorted

        # Keep current selection if still exists; otherwise pick first
        cur = self.port_var.get().strip()
        if cur in port_names_sorted:
            self.port_combo.set(cur)
        elif port_names_sorted:
            self.port_combo.set(port_names_sorted[0])
            self.port_var.set(port_names_sorted[0])

        # Log what we found (optional but helpful)
        self.append_log("Ports found:")
        for p in ports_sorted:
            self.append_log(f"  {p.device} - {p.description}")

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

        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get())
        except Exception:
            messagebox.showerror("Error", "Baud must be a number (e.g., 115200).")
            return

        if not port:
            messagebox.showerror("Error", "Please select a COM port.")
            return

        self.status_var.set(f"Connecting to {port} @ {baud}...")
        self.append_log(f"Trying to connect to {port} @ {baud}...")

        self.connect_btn.configure(state="disabled")

        def worker():
            try:
                # Note: SPP ignores baud in many stacks, but it's fine to set it.
                ser = serial.Serial(port, baudrate=baud, timeout=1)
                # Give Windows/driver a moment to settle
                time.sleep(0.2)
                self.ser = ser
                self.root.after(0, lambda: self.on_connected(port, baud))
            except Exception as e:
                self.root.after(0, lambda err=e: self.on_connect_failed(err))

        threading.Thread(target=worker, daemon=True).start()

    def on_connected(self, port, baud):
        self.set_connected_ui(True)
        self.status_var.set(f"Connected to {port} @ {baud}.")
        self.append_log("Connected!")

    def on_connect_failed(self, e: Exception):
        self.ser = None
        self.set_connected_ui(False)
        self.status_var.set("Not connected.")
        self.append_log(f"Failed to connect: {e}")
        messagebox.showerror("Connection failed", str(e))
        self.connect_btn.configure(state="normal")

    def disconnect(self):
        if not self.connected:
            return
        try:
            if self.ser:
                self.append_log("Disconnecting...")
                try:
                    self.ser.close()
                except Exception:
                    pass
        except Exception as e:
            self.append_log(f"Error while disconnecting: {e}")
        finally:
            self.ser = None
            self.set_connected_ui(False)
            self.status_var.set("Not connected.")
            self.append_log("Disconnected.")

    def send_cmd(self, cmd: str):
        if not self.connected or not self.ser:
            return
        if cmd not in ("l", "r", "s", "0"):
            return

        def worker():
            with self.sending_lock:
                try:
                    # Match your working script: send a single character
                    self.ser.write(cmd.encode("utf-8"))
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
