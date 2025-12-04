import bluetooth
import time

# the MAC address of our ESP32 is here
ESP32_ADDR = "B8:D6:1A:5B:DE:BA"

print(f"Trying to connect to ESP32 at {ESP32_ADDR}...")
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

try:
    sock.connect((ESP32_ADDR, 1))   # RFCOMM channel 1
    print("✅ Connected to ESP32!")
except Exception as e:
    print("❌ Failed to connect:", e)
    exit()

print("You can now type commands:")
print("  l = turn left")
print("  r = turn right")
print("  s = straight") # move from 0
print("  0 = calibrate")
print("  q = quit")

try:
    while True:
        cmd = input("> ").strip().lower()

        if cmd == 'q':
            print("Disconnecting...")
            break

        if cmd not in ['l', 'r', 's', '0']:
            print("Invalid command.")
            continue

        sock.send(cmd)
        time.sleep(0.05)
except KeyboardInterrupt:
    pass

sock.close()
print("Connection closed.")
