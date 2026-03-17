# 🐍 SnakeRobot

A Bluetooth-controlled snake robot built on the **ESP32** microcontroller. Five servo motors create a sinusoidal traveling-wave gait, and a Python GUI lets you steer, tune amplitude, and watch live telemetry — all over a wireless Bluetooth SPP link.

---

## Table of Contents
- [Demo / Pinout](#demo--pinout)
- [How It Works](#how-it-works)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [Bluetooth Commands](#bluetooth-commands)
- [Python GUI](#python-gui)
- [Project Structure](#project-structure)

---

## Demo / Pinout

![ESP32 pinout reference](https://europe1.discourse-cdn.com/arduino/optimized/4X/a/b/8/ab801a37c8d5240f8142c22b6a55fc02a6428467_2_689x399.png)

---

## How It Works

Each joint of the snake is driven by a servo. The firmware generates a **sinusoidal traveling wave** across all five joints:

```
angle_i(t) = center + turnOffset + amplitude × sin(2π × frequency × t + i × 2π/5)
```

- **Frequency** is fixed at 0.5 Hz (one full undulation every 2 seconds).
- **Amplitude** (0 – 60 °) controls how wide each swing is — higher = faster forward motion.
- **Turn offset** shifts the neutral position of all joints together, steering the robot left or right.
- Both amplitude and turn offset are **slew-rate limited** so changes are smooth, not jerky.

The ESP32 also streams back telemetry (heading, X position, Y position) as packed `float32` frames so the Python GUI can display live position data.

---

## Hardware Setup

### Parts
| Qty | Component |
|-----|-----------|
| 1 | ESP32 development board |
| 5 | Servo motors (standard 5 V hobby servos) |
| 1 | 5 V regulated power supply (servos draw significant current) |
| — | Jumper wires, chassis / 3-D printed links |

### Wiring

| Servo # | ESP32 GPIO |
|---------|-----------|
| 1 | 13 |
| 2 | 12 |
| 3 | 14 |
| 4 | 27 |
| 5 | 26 |
| 6 | 25 |
| 7 | 32 |
| 8 | 2  |

> **Tip:** Power the servos from a dedicated 5 V rail, **not** directly from the ESP32's 3.3 V pin. Share a common ground between the servo power supply and the ESP32.

### Motor Calibration
Before running the snake gait, make sure each servo is centered at 90 °:

1. Flash the firmware and open the Python GUI (or a serial terminal).
2. Send the `c` command — this drives all servos to 90 ° and stops motion.
3. Physically adjust the servo horns / linkages so the robot is straight.
4. Send `s` to resume motion.

---

## Software Setup

### ESP32 Firmware

1. Install the **Arduino IDE** and add the ESP32 board package:  
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Install the **ESP32Servo** library via the Arduino Library Manager.
3. Clone or copy the `BluetoothSerial` header into your sketch folder:
   ```cpp
   #include "BluetoothSerial.h"
   ```
4. Open `ESP32/Test_Code_260119.cpp` (the latest version) in the Arduino IDE, select your ESP32 board and COM port, then upload.

### Python GUI

Requires **Python 3.7+** and a Bluetooth-paired serial port.

```bash
pip install pyserial
# tkinter ships with most Python distributions; if missing:
# Ubuntu/Debian: sudo apt-get install python3-tk
# Windows/macOS: included in the standard Python installer
```

Pair your PC with the ESP32 Bluetooth device named **`ESP32_Snake`**, note the assigned COM port (e.g. `COM11` on Windows, `/dev/rfcomm0` on Linux), then edit the `PORT` constant at the top of `bluetooth.py` or `bluetooth_receive.py`.

---

## Bluetooth Commands

The GUI sends single-character (or 8-byte padded) commands over the Bluetooth SPP link.

| Key / Command | Action |
|---------------|--------|
| `s` | Go **straight** — reset turn offset to 0 and resume motion |
| `l` | Turn **left** (smooth) |
| `r` | Turn **right** (smooth) |
| `c` | **Calibrate** — move all servos to 90 ° and stop motion |
| `0` – `9` | Set amplitude to `digit × 5` degrees (0 – 45 °) |
| `+` | Increase amplitude by 5 ° |
| `-` | Decrease amplitude by 5 ° |

---

## Python GUI

Two GUI scripts are included:

### `bluetooth_receive.py` — Simple Telemetry Viewer (For testing only)
- Connects to the ESP32 over a serial COM port.
- Displays live **direction (°)**, **X position (m)**, and **Y position (m)** from the ESP32 telemetry stream.
- Lets you type and send a command character to the robot.

### `bluetooth.py` — Full Control Panel
- Everything in the simple viewer, plus:
  - **D-pad** for steering, with drag support and auto-repeat.
  - **Amplitude ±** buttons with a live display.
  - **Mode selector**: Manual, Direction auto-control, Position auto-control.
  - Analog steering angle input.

**Running either script:**
```bash
python bluetooth.py
# or
python bluetooth_receive.py
```

---

## Project Structure

```
SnakeRobot/
├── Arduino/
│   └── Test_Code_250902/          # Early 2-servo Arduino prototype
├── ESP32/
│   ├── Manual_Control_SerialBT/   # Basic manual Bluetooth control
│   ├── Proportional_control/      # Proportional steering controller
│   ├── Sensors_with_Manual_Control/
│   ├── Sensors_with_Manual_Control_with_bluetooth_send_receive/
│   ├── Test_Code_251028.ino       # Iterative firmware versions
│   ├── Test_Code_251118.ino
│   ├── Test_Code_251130.cpp
│   ├── Test_Code_251202.cpp
│   ├── Test_Code_260109.cpp
│   ├── Test_Code_260114.cpp
│   ├── Test_Code_260119/
│   └── Test_Code_260119.cpp       # ← Latest firmware (use this one)
├── bluetooth.py                   # Full-featured Python control GUI
├── bluetooth_receive.py           # Lightweight telemetry viewer
└── README.md
```
