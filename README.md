# 🐍 SnakeRobot

A snake robot built on the **ESP32** microcontroller. Eight servo motors create a sinusoidal traveling-wave gait, controlled via Bluetooth from a Python GUI that supports manual D-pad steering, autonomous direction tracking, and autonomous position control — with live telemetry (heading, X, Y) streamed back from an onboard IMU and optical-flow sensor.

---

## Table of Contents
- [Design Fair Poster](#design-fair-poster)
- [Demo / Pinout](#demo--pinout)
- [How It Works](#how-it-works)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [Bluetooth Commands](#bluetooth-commands)
- [Python GUI](#python-gui)
- [Camera Web Server](#camera-web-server)
- [3D Printing](#3d-printing)
- [Project Structure](#project-structure)

---

## Design Fair Poster

![Design Fair](https://github.com/GaryZhous/SnakeRobot/main/image.png)

The poster provides a high-level overview of the project — motivation, system architecture, hardware components, control strategy, and results — as presented at the design fair.

---

## Demo / Pinout

![ESP32 pinout reference](https://europe1.discourse-cdn.com/arduino/optimized/4X/a/b/8/ab801a37c8d5240f8142c22b6a55fc02a6428467_2_689x399.png)

---

## How It Works

Each joint of the snake is driven by a servo. The firmware generates a **sinusoidal traveling wave** across all eight joints:

```
angle_i(t) = center + turnOffset + amplitude × sin(2π × frequency × t + i × 2π/8)
```

- **Frequency** is fixed at 0.5 Hz (one full undulation every 2 seconds).
- **Amplitude** (0 – 60 °) controls how wide each swing is — higher = faster forward motion.
- **Turn offset** shifts the neutral position of all joints together, steering the robot left or right.
- Both amplitude and turn offset are **slew-rate limited** so changes are smooth, not jerky.

The ESP32 reads heading from a **BNO08x IMU** and displacement from a **PMW3901 optical-flow sensor**, fusing them into world-frame odometry. This telemetry (heading, X position, Y position) is streamed back as packed `float32` frames so the Python GUI can display live position data and close a control loop.

---

## Hardware Setup

### Parts
| Qty | Component |
|-----|-----------|
| 1 | ESP32 development board |
| 8 | Servo motors (standard 5 V hobby servos) |
| 1 | BNO08x IMU (SPI) |
| 1 | Bitcraze PMW3901 optical-flow sensor (SPI) |
| 1 | 5 V regulated power supply (servos draw significant current) |
| — | Jumper wires, chassis / 3-D printed links (see [3D Printing](#3d-printing)) |

### Wiring

| Servo # | ESP32 GPIO |
|---------|-----------|
| 1 | 13 |
| 2 | 12 |
| 3 | 14 |
| 4 | 27 |
| 5 | 26 |
| 6 | 25 |
| 7 | 33 |
| 8 | 32 |

| Sensor | Signal | ESP32 GPIO |
|--------|--------|-----------|
| BNO08x | CS | 15 |
| BNO08x | INT | 22 |
| BNO08x | RESET | 4 |
| PMW3901 | CS | 5 |
| Both | SCK / MISO / MOSI | 18 / 19 / 23 |

> **Tip:** Power the servos from a dedicated 5 V rail, **not** directly from the ESP32's 3.3 V pin. Share a common ground between the servo power supply and the ESP32.

### Motor Calibration
Before running the snake gait, make sure each servo is centered at 90 °:

1. Flash the `Calibration_Serial` sketch and open the Serial Monitor.
2. The sketch drives all servos to 90 ° sequentially so you can verify each joint.
3. Physically adjust the servo horns / linkages so the robot is straight.
4. Flash the main firmware and send `c` over Bluetooth to re-center before motion.

---

## Software Setup

### ESP32 Firmware

1. Install the **Arduino IDE** and add the ESP32 board package:  
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Install the following libraries via the Arduino Library Manager:
   - **ESP32Servo**
   - **Adafruit BNO08x** (IMU)
   - **Bitcraze PMW3901** (optical flow)
3. Open `ESP32/6_Position_Control_Avg_Yaw/` (the latest version) in the Arduino IDE, select your ESP32 board and COM port, then upload.

### Python GUI

Requires **Python 3.7+** and a Bluetooth-paired serial port.

```bash
pip install pyserial
# tkinter ships with most Python distributions; if missing:
# Ubuntu/Debian: sudo apt-get install python3-tk
# Windows/macOS: included in the standard Python installer
```

Pair your PC with the ESP32 Bluetooth device named **`ESP32_Snake`**, note the assigned COM port (e.g. `COM11` on Windows, `/dev/rfcomm0` on Linux), then edit the `PORT` constant at the top of `bluetooth.py`.

---

## Bluetooth Commands

All commands are sent as **exactly 8 ASCII bytes** over the Bluetooth SPP link (1 character header + 7 zero-padding bytes, e.g. `"l0000000"`).

### Mode markers
| Command | Action |
|---------|--------|
| `M0000000` | Switch to **Manual** mode |
| `D0000000` | Switch to **Direction** auto-control (tracks a target heading) |
| `P0000000` | Switch to **Position** auto-control (tracks target X/Y coordinates) |

### Motion commands (Manual mode)
| Key / Command | Action |
|---------------|--------|
| `s` | Go **straight** — reset turn offset to 0 and resume motion |
| `l` | Turn **left** (smooth) |
| `r` | Turn **right** (smooth) |
| `c` | **Calibrate** — move all servos to 90 ° and stop motion |
| `0` – `9` | Set amplitude to `digit × 5` degrees (0 – 45 °) |
| `+` | Increase amplitude by 5 ° (up to 60 °) |
| `-` | Decrease amplitude by 5 ° |

### Auto-control payloads
| Format | Mode | Meaning |
|--------|------|---------|
| `+1200000` / `-0900000` | Direction | Target heading angle in degrees (signed, 8 chars) |
| `+100+200` | Position | Target X and Y position in cm (signed 3-digit each) |

---

## Python GUI

### `bluetooth.py` — Full Control Panel

- Connects to the ESP32 over a Bluetooth SPP serial port.
- Displays live **direction (°)**, **X position**, and **Y position** from the telemetry stream.
- **D-pad** for manual steering with drag support, auto-repeat, and analog angle input (−15 ° … +15 °).
- **Amplitude ±** buttons with a live display (range 0 – 60 °).
- **Mode selector** with three modes:
  - **Manual** — direct D-pad / key control.
  - **Direction** (auto) — enter a target heading; the robot steers to match it.
  - **Position** (auto) — enter target X/Y coordinates; the robot navigates to them.

**Running the script:**
```bash
python bluetooth.py
```

---

## Camera Web Server

The `CameraWebServer/` folder contains an **ESP32-CAM** sketch that streams live video over Wi-Fi.

### Setup
1. Open `CameraWebServer/CameraWebServer.ino` in the Arduino IDE.
2. Edit the `ssid` and `password` constants at the top of the file to match your Wi-Fi network.
3. Select the correct camera model by editing `board_config.h`.
4. Upload to an ESP32-CAM module.
5. Open the Serial Monitor at 115200 baud — after connecting the camera prints its local IP address.
6. Navigate to `http://<IP>` in a browser to view the live stream and camera controls.

---

## 3D Printing

The `CAD Files/` directory contains all STL meshes used to print the robot chassis, and the `3D Print Files/` directory contains pre-sliced Cura `.3mf` files ready to send directly to a printer.

### CAD Files (STL)

| File | Description |
|------|-------------|
| `Assembled.stl` | Full snake assembly reference |
| `AssembledHead.stl` / `AssembledTail.stl` | Head and tail sub-assemblies |
| `Head.stl` / `Tail.stl` | Individual head and tail prints |
| `Connecting Piece.stl` | Body links that join servo segments |
| `Motor Wrap.stl` | Servo motor sleeve / housing |
| `MG996R SBORKA.stl` | MG996R servo bracket assembly |
| `Clamp.stl` | Clamping ring for segment joints |
| `ESPBracket.stl` | ESP32 board mounting bracket |
| `ESP-WROOM32.stl` | ESP32 module reference model |
| `IMU.stl` | BNO08x IMU mounting housing |
| `Optical Flow Senssor.stl` | PMW3901 optical-flow sensor mount |
| `Battery.stl` / `BatteryBracket.stl` / `BatteryHead.stl` / `Battery Compartment.stl` | Battery enclosure and mounting hardware |
| `CamCaseTop.stl` / `CamCase Bottom.stl` | ESP32-CAM enclosure |
| `CameraPlatform 2.stl` | Camera platform for top-down mounting |

### 3D Print Files (pre-sliced)

Ready-to-print Cura `.3mf` files for **PLA** at standard settings:

| File | Contents |
|------|----------|
| `Rev3 Segment.gcode.3mf` | Snake body link segments |
| `Rev3 Brackets PLA.gcode.3mf` | Servo mounting brackets |
| `Rev3 Head PLA.gcode.3mf` | Head section |
| `Rev3 Tail PLA.gcode.3mf` | Tail section |
| `Rev3 BatteryBracket PLA.gcode.3mf` | Battery bracket |
| `Rev3 CameraPlatform PLA.gcode.3mf` | Camera platform |
| `Servo Base Plate (1).gcode.3mf` | Servo base plate |

> **Tip:** Open `.3mf` files in [Ultimaker Cura](https://ultimaker.com/software/ultimaker-cura/) (or any slicer that supports `.3mf`) to preview or adjust print settings before slicing.

---

## Project Structure

```
SnakeRobot/
├── Team128_Design_Fair_Poster.pdf         # Design fair poster
├── Arduino/
│   └── Test_Code_250902/              # Early 2-servo Arduino prototype
├── CAD Files/                         # STL meshes for all printed parts
│   ├── Assembled.stl                  # Full snake assembly reference
│   ├── Head.stl / Tail.stl
│   ├── Connecting Piece.stl
│   ├── Motor Wrap.stl
│   ├── ESPBracket.stl
│   ├── IMU.stl / Optical Flow Senssor.stl
│   ├── Battery Compartment.stl / Battery.stl / ...
│   └── CamCaseTop.stl / CamCase Bottom.stl / ...
├── 3D Print Files/                    # Pre-sliced Cura .3mf files (PLA)
│   ├── Rev3 Segment.gcode.3mf
│   ├── Rev3 Brackets PLA.gcode.3mf
│   ├── Rev3 Head PLA.gcode.3mf
│   ├── Rev3 Tail PLA.gcode.3mf
│   ├── Rev3 BatteryBracket PLA.gcode.3mf
│   ├── Rev3 CameraPlatform PLA.gcode.3mf
│   └── Servo Base Plate (1).gcode.3mf
├── CameraWebServer/                   # ESP32-CAM Wi-Fi video stream server
│   ├── CameraWebServer.ino
│   ├── board_config.h
│   ├── camera_pins.h
│   └── ...
├── ESP32/
│   ├── Manual_Control_SerialBT/       # Basic manual Bluetooth control
│   ├── 1_Sensors_with_Manual_Control/
│   ├── 2_Sensors_with_Manual_Control_with_bluetooth_send_receive/
│   ├── 3_Proportional_control/        # Proportional steering controller
│   ├── 4_Analog_Manual/               # Analog angle steering via Bluetooth
│   ├── 5_Analog_Manual_Position_Control/  # Analog steering + position control
│   ├── 6_Position_Control_Avg_Yaw/    # ← Latest firmware (use this one)
│   ├── Calibration/                   # Servo calibration sketch
│   ├── Calibration_Serial/            # Serial-assisted servo calibration
│   ├── Tiger_avg_yaw/                 # Alternate position-control variant
│   ├── Test_Code_251028/
│   ├── Test_Code_251118.ino
│   ├── Test_Code_251130.cpp
│   ├── Test_Code_251202.cpp
│   ├── Test_Code_260109.cpp
│   ├── Test_Code_260114.cpp
│   ├── Test_Code_260119/              # Sensor integration test sketch
│   └── Test_Code_260119.cpp
├── bluetooth.py                       # Full-featured Python control GUI
└── README.md
```
