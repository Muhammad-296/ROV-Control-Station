<div align="center">

# ROV CONTROL STATION

### Six-Thruster Underwater Robotics · Integrated V5

**Python / PySide6 · Arduino Mega 2560 · PS4 · RTSP · VTK**

[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![GUI](https://img.shields.io/badge/GUI-PySide6-41CD52?logo=qt&logoColor=white)](https://doc.qt.io/qtforpython-6/)
[![Firmware](https://img.shields.io/badge/Firmware-Arduino%20Mega-00979D?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Protocol](https://img.shields.io/badge/Serial%20Protocol-V5-6554C0)](#)
[![Status](https://img.shields.io/badge/Hardware%20Testing-Required-orange)](#safety)

**A desktop mission-control dashboard for a six-thruster ROV,
with dedicated motor verification, live telemetry, and video streaming.**

[Features](#features) · [Quick start](#quick-start) · [Hardware](#hardware) · [Safety](#safety) · [Changelog](#changelog)

</div>

---

## Features

| Subsystem | Capabilities |
|:--|:--|
| **Mission control** | PySide6 dashboard, PS4 and keyboard input, 5-DOF thruster mixing |
| **Motor Test Station** | Independent M0–M5 momentary tests, speed and direction controls, ACK tracking, event CSV |
| **Camera** | RTSP stream, separate camera window, transport/profile options |
| **3D & sensors** | STL model viewer, MPU6050 attitude, DS18B20 temperature, CSV logging |
| **Safety logic** | Firmware-controlled arming, latched emergency stop, mode isolation, motor-test watchdog |

> [!IMPORTANT]
> This is a robotics development project, **not a safety-certified control system**. Software checks and simulated tests do not establish safe performance on physical hardware.

## Quick start

**Requirements:** Windows, Python 3.12, Arduino Mega 2560, a compatible USB-to-TTL adapter for runtime Serial1 communications, and the hardware described below.

Run from PowerShell in the project folder:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe ROV_Control_Station_Integrated.py
```

Alternatively run `INSTALL_WINDOWS.ps1`, then `RUN_GUI.bat`. The additional `START_ROV_WINDOWS.bat` and `START_ROV_MAC.command` are included as optional launchers; macOS behavior has not been independently verified. For building a Windows executable, run `BUILD_EXE.ps1` after installation. Use **PySide6 only** in the build environment; mixing PyQt6 and PySide6 can break PyInstaller builds.

### Firmware

Open `ROV_Mega_Integrated/ROV_Mega_Integrated.ino` in Arduino IDE, select **Arduino Mega or Mega 2560**, install **OneWire** and **DallasTemperature**, and upload using the Mega's programming USB port **while motor power is disconnected**. Firmware and GUI must both be **protocol V5**.

## Hardware

| Device | Arduino Mega pins |
|:--|:--|
| M0–M5 RPWM | 2, 4, 6, 8, 10, 12 |
| M0–M5 LPWM | 3, 5, 7, 9, 11, 13 |
| M0–M5 R_EN | 23, 27, 31, 35, 39, 43 |
| M0–M5 L_EN | 25, 29, 33, 37, 41, 45 |
| Serial1 TX1 / RX1 | 18 / 19 |
| MPU6050 SDA / SCL | 20 / 21 |
| DS18B20 | A15 (4.7 kΩ pull-up) |
| Relay channels | 17, 16, 15 |

**Thruster order:** M0 Front Left, M1 Front Right, M2 Rear Left, M3 Rear Right, M4 Vertical Left, M5 Vertical Right. Confirm against actual wiring before powering motors.

USB-TTL adapter: **TXD → Mega RX1 (19)**, **RXD ← Mega TX1 (18)**, and **GND ↔ GND**. Ensure adapter RX input is safe for a 5 V Mega TX level. The GUI uses the adapter COM port at **115200 baud**.

## Motor-test workflow

1. Remove propellers and secure the system. Provide an independent physical emergency cutoff.
2. Connect and confirm the firmware handshake `SYS:HELLO,5,ROV_MEGA6`.
3. While disarmed, choose motor-test mode and open the separate Motor Test Station.
4. Enable motor testing, then hold keys **1–6** (M0–M5). **Release** to command zero PWM.
5. **Space:** stop and disarm. **Esc:** latched emergency stop.

Test mode enforces one motor command at a time, a 60% requested-power ceiling, a 225 ms active-test watchdog, and an 8-second continuous-test limit. These are backup protections, not substitutes for electrical isolation.

## Protocol

| Command | Role |
|:--|:--|
| `SYS,HELLO,5` | Verify matching V5 firmware |
| `SYS,MODE,DRIVE` / `SYS,MODE,INDIVIDUAL` | Select operating mode while disarmed |
| `SYS,ARM` / `SYS,DISARM` | Authorization control |
| `SYS,ESTOP` / `SYS,RESET` | Latched emergency stop and explicit reset |
| `MT,1,0,15` | Motor-test sequence 1, M0, +15% |
| `MT,2,-1,0` | Stop all motor-test outputs |
| `PING` | Link test (`PONG`) |

Firmware acknowledgements confirm a command was handled; they **do not measure physical thruster motion**.

## Project files

```text
ROV-Control-Station/
├── ROV_Control_Station_Integrated.py   # Main PySide6 GUI
├── ROV_Mega_Integrated/
│   └── ROV_Mega_Integrated.ino         # Matching Mega firmware V5
├── models/ROV Assembly_Fully.STL      # CAD model (large file)
├── docs/CHANGES_V3.md                 # Release history
├── docs/CHANGES_V4.md
├── docs/CHANGES_V5.md
├── test_protocol.py                   # Software test
├── requirements.txt
├── INSTALL_WINDOWS.ps1
├── RUN_GUI.bat
├── BUILD_EXE.ps1
├── BUILD_ROV_EXE.bat
├── START_ROV_WINDOWS.bat
├── START_ROV_MAC.command
└── .gitignore
```

**Not committed:** packaged executable `ROV.exe` (~203 MB), local PyInstaller logs, caches, virtual environments, credentials, and generated builds. Distribute executable builds through GitHub Releases or an external download rather than Git history. The CAD STL is ~68 MB, so Git LFS is recommended if it will change frequently.

## Tests

```powershell
py -3.12 -m py_compile ROV_Control_Station_Integrated.py
py -3.12 test_protocol.py
```

Software protocol checks do not constitute Arduino IDE compilation or real-device commissioning.

## Safety

> [!CAUTION]
> **Remove propellers** for initial motor tests. Install a **hardwired emergency power cutoff**, limit supply current, confirm shared signal grounds, check pin assignments and relay polarity, and verify each motor independently. A zero PWM command is not proof of mechanical stop. Never operate an ROV in water based on software tests alone.

## Changelog

- [V3 — Individual motor controls](docs/CHANGES_V3.md)
- [V4 — Dedicated test mode](docs/CHANGES_V4.md)
- [V5 — Advanced Motor Test Station](docs/CHANGES_V5.md)

## Attribution

The source GUI references a DualShock 4 image attributed to **ITEagle Europe / Sebastiaan Broekhoven**, under **CC BY-SA 4.0**. See the attribution and source URL in the Python source header. Check rights for all models, images and third-party assets before publication.

**Maintainer:** [Muhammad-296](https://github.com/Muhammad-296)

**License:** No project-wide open-source license has been declared. All rights reserved unless a license is added by the owner.
