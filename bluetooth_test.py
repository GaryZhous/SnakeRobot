import struct
import threading
import time
import serial

# =========================
# Configure these
# =========================
PORT = "COM10"      # <-- your Bluetooth SPP COM port
BAUD = 115200       # SPP often ignores baud, but keep consistent
READ_TIMEOUT = 0.2  # seconds

# Telemetry format: 3 float32 little-endian = 12 bytes
TELEM_FMT = "<fff"
TELEM_SIZE = struct.calcsize(TELEM_FMT)  # 12


def rx_loop(ser: serial.Serial, stop_flag: threading.Event):
    """
    Continuously read telemetry frames from ESP32.
    Each frame is exactly 12 bytes: direction_deg, xw_m, yw_m (float32 LE).
    """
    print("[RX] Telemetry receiver started.")
    while not stop_flag.is_set():
        try:
            data = ser.read(TELEM_SIZE)
            if len(data) != TELEM_SIZE:
                continue  # timeout, try again

            direction_deg, x_m, y_m = struct.unpack(TELEM_FMT, data)
            print(f"[TEL] dir={direction_deg:7.2f} deg | x={x_m:8.4f} m | y={y_m:8.4f} m")

        except (serial.SerialException, OSError) as e:
            print(f"[RX] Serial error: {e}")
            break
        except struct.error as e:
            # If bytes ever get misaligned, unpack can fail.
            # With your current ESP code (no header), this is rare but possible.
            print(f"[RX] Unpack error (possible misalignment): {e}")
            # Optional: flush and continue
            try:
                ser.reset_input_buffer()
            except Exception:
                pass

    print("[RX] Telemetry receiver stopped.")


def main():
    ser = serial.Serial(PORT, BAUD, timeout=READ_TIMEOUT)
    time.sleep(0.2)
    print(f"Connected to {PORT}.")
    print("Commands: l r s c 0..9 + -")
    print("Type a command and press Enter. Type 'q' to quit.\n")

    stop_flag = threading.Event()
    t = threading.Thread(target=rx_loop, args=(ser, stop_flag), daemon=True)
    t.start()

    try:
        while True:
            cmd = input("> ").strip()
            if not cmd:
                continue
            if cmd.lower() == "q":
                break

            c = cmd[0]
            if c not in "lrs c0123456789+-".replace(" ", ""):
                print("Invalid. Use: l r s c 0..9 + -")
                continue

            ser.write(c.encode("utf-8"))
            print(f"[TX] Sent: {c}")

    finally:
        stop_flag.set()
        time.sleep(0.1)
        try:
            ser.close()
        except Exception:
            pass
        print("Disconnected.")


if __name__ == "__main__":
    main()
