import struct
import threading
import time
import queue
import serial
import tkinter as tk
from tkinter import ttk, messagebox

# =========================
# Configure these
# =========================
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
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Snake Telemetry")
        self.root.geometry("420x260")

        self.ser = None
        self.stop_flag = threading.Event()
        self.rx_thread = None
        self.q = queue.Queue()

        # -------- UI Variables --------
        self.port_var = tk.StringVar(value=PORT)
        self.dir_var  = tk.StringVar(value="--")
        self.x_var    = tk.StringVar(value="--")
        self.y_var    = tk.StringVar(value="--")
        self.status_var = tk.StringVar(value="Disconnected")

        # -------- Layout --------
        pad = {"padx": 10, "pady": 8}

        top = ttk.Frame(root)
        top.pack(fill="x", **pad)

        ttk.Label(top, text="COM Port:").pack(side="left")
        self.port_entry = ttk.Entry(top, textvariable=self.port_var, width=12)
        self.port_entry.pack(side="left", padx=(6, 10))

        self.btn_connect = ttk.Button(top, text="Connect", command=self.connect)
        self.btn_connect.pack(side="left", padx=(0, 6))

        self.btn_disconnect = ttk.Button(top, text="Disconnect", command=self.disconnect, state="disabled")
        self.btn_disconnect.pack(side="left")

        mid = ttk.Frame(root)
        mid.pack(fill="both", expand=True, **pad)

        # Telemetry boxes (read-only entries)
        self._make_row(mid, "Direction (deg):", self.dir_var, row=0)
        self._make_row(mid, "Pos X (m):",       self.x_var,   row=1)
        self._make_row(mid, "Pos Y (m):",       self.y_var,   row=2)

        # Command send row
        cmd_frame = ttk.Frame(mid)
        cmd_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        cmd_frame.columnconfigure(1, weight=1)

        ttk.Label(cmd_frame, text="Command:").grid(row=0, column=0, sticky="w")
        self.cmd_entry = ttk.Entry(cmd_frame)
        self.cmd_entry.grid(row=0, column=1, sticky="ew", padx=(6, 6))
        self.cmd_entry.bind("<Return>", lambda e: self.send_command())

        self.btn_send = ttk.Button(cmd_frame, text="Send", command=self.send_command, state="disabled")
        self.btn_send.grid(row=0, column=2, sticky="e")

        # Status bar
        bottom = ttk.Frame(root)
        bottom.pack(fill="x", padx=10, pady=(0, 10))
        ttk.Label(bottom, textvariable=self.status_var).pack(side="left")

        # Poll queue periodically for new telemetry
        self.root.after(50, self._poll_queue)

        # Proper close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _make_row(self, parent, label, var, row: int):
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=6)
        e = ttk.Entry(parent, textvariable=var, state="readonly", width=20, justify="center")
        e.grid(row=row, column=1, sticky="ew", padx=(10, 0), pady=6)

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
        self.cmd_entry.focus_set()

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

    def send_command(self):
        if self.ser is None:
            return

        text = self.cmd_entry.get().strip()
        if not text:
            return

        c = text[0]  # one byte/letter at a time
        if c not in "lrs c0123456789+-".replace(" ", ""):
            messagebox.showwarning("Invalid Command", "Valid: l r s c 0..9 + -")
            return

        try:
            self.ser.write(c.encode("utf-8"))
            self.status_var.set(f"Sent: {c}")
        except Exception as e:
            messagebox.showerror("Send Failed", str(e))

        self.cmd_entry.delete(0, "end")

    def _poll_queue(self):
        # Consume all queued telemetry updates
        try:
            while True:
                item = self.q.get_nowait()

                if isinstance(item, tuple) and len(item) == 3 and item[0] == "__ERROR__":
                    self.status_var.set(f"Serial error: {item[1]}")
                    self.disconnect()
                    break

                if isinstance(item, tuple) and len(item) == 3 and item[0] == "__STOP__":
                    break

                direction_deg, x_m, y_m = item
                self.dir_var.set(f"{direction_deg:0.2f}")
                self.x_var.set(f"{x_m:0.4f}")
                self.y_var.set(f"{y_m:0.4f}")

        except queue.Empty:
            pass

        self.root.after(50, self._poll_queue)

    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    # Nice default theme on Windows
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass

    TelemetryUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
