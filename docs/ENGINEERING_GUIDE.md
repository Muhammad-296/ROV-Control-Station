# Engineering handbook — V5

## Scope and system design

The ROV uses a **Python 3.12 / PySide6** Windows operator console, **Arduino Mega 2560** firmware over hardware **Serial1** at **115200 baud**, six **BTS7960** DC motor drivers, six thrusters, an MPU6050 IMU, a DS18B20 water-temperature probe and three relay outputs. RTSP video and a local STL viewer live in the desktop GUI. Both GUI and firmware must speak **V5**; earlier versions are not protocol-compatible.

## Motor driver pin map

| Motor | Location | RPWM | LPWM | R_EN | L_EN |
|:--|:--|--:|--:|--:|--:|
| M0 | Front Left | 2 | 3 | 23 | 25 |
| M1 | Front Right | 4 | 5 | 27 | 29 |
| M2 | Rear Left | 6 | 7 | 31 | 33 |
| M3 | Rear Right | 8 | 9 | 35 | 37 |
| M4 | Vertical Left | 10 | 11 | 39 | 41 |
| M5 | Vertical Right | 12 | 13 | 43 | 45 |

Names are firmware/GUI conventions; confirm against actual wiring. Directions can be reversed by the firmware's `MOTOR_DIR` values or GUI's test inversion setting, as appropriate. Verify each at low power with propellers removed.

## Peripheral connections

| Peripheral | Arduino Mega connection | Notes |
|:--|:--|:--|
| Serial1 TX | D18 → TTL adapter RXD | Check adapter receive voltage tolerance; Mega output is 5 V logic |
| Serial1 RX | D19 ← TTL adapter TXD | Common ground required |
| MPU6050 | SDA 20 / SCL 21 | I2C; verify address and module supply requirements |
| DS18B20 | A15 data | 4.7 kΩ data-to-5 V pull-up per firmware notes |
| Solenoid valve relay | D17 | Relay 0 |
| Light relay | D16 | Relay 1 |
| Alert relay | D15 | Relay 2 |

The firmware currently declares `RELAY_ACTIVE_HIGH = true`; physical relay boards can differ. Do not assume the configured polarity is safe without inspection. Keep motor power separate from USB serial logic and implement a reachable hardware power cutoff.

## Desktop software

- `ROV_Control_Station_Integrated.py`: dashboard; keyboard/PS4 movement; motor-test window; COM connection and handshake; camera; sensors; STL visualization; logs.
- `requirements.txt`: PySide6, pygame, headless OpenCV, numpy, vtk, PyInstaller; use Python 3.12.
- `test_protocol.py`: isolated behavior tests for test-mode packets, mode restrictions, release and stop.
- `BUILD_EXE.ps1` / `BUILD_ROV_EXE.bat`: Windows packaging. Avoid mixing PyQt6 and PySide6 in a single build environment.

## Firmware behavior

The MCU owns arming authorization, E-STOP/FAILSAFE state, mode gating and PWM. The independent motor-test path uses a sequence-bearing `MT` packet and returns `MT:ACK`. The firmware has a **225 ms active test packet watchdog** and an **8 s continuous-test limit**; these are software safeguards, not guaranteed physical stops. A zero PWM command may allow motor coasting.

## Installation

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe ROV_Control_Station_Integrated.py
```

For firmware, install `OneWire` and `DallasTemperature` in Arduino IDE, choose **Arduino Mega 2560**, and upload `ROV_Mega_Integrated/ROV_Mega_Integrated.ino` over the board's programming USB cable while motor load power is disconnected. For GUI communication, choose the **USB–TTL adapter COM port** connected to Serial1 (not the board's USB programming port).

## Operational sequence

1. Remove propellers for bench diagnostics; secure the frame and provide a physical cutoff.
2. Inspect polarity, shared signal ground, logic voltage, supply limits and each motor-to-pin mapping.
3. Connect the adapter and validate `SYS:HELLO,5,ROV_MEGA6`.
4. In **DRIVE** mode, confirm intended motion and stop behavior at low power.
5. In **INDIVIDUAL** mode, explicitly arm motor testing; hold **1–6** for M0–M5 and release to stop. Space disarms; Escape latches E-STOP.
6. Check logged test observations, watchdog handling, relay polarity and sensor data.
7. Record actual test configuration and observations in [Hardware validation](HARDWARE_VALIDATION.md).

## Design limitations

- Sensor telemetry does not give absolute underwater position; gyro-based yaw can drift.
- PWM feedback reports commands, not measured RPM, thrust or current.
- Camera credentials belong in runtime settings, not source code, workflow files or Git history.
- Software tests and CI compilation do not establish waterproofing, electrical isolation or human safety.
