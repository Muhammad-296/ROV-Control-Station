# Troubleshooting guide

| Symptom | Check | Next step |
|:--|:--|:--|
| GUI COM port opens, no firmware identity | Correct USB–TTL port, 115200 baud, TXD→D19 RX1, RXD←D18 TX1, common GND, logic voltage | Check for `SYS:HELLO,5,ROV_MEGA6`; do not arm until confirmed |
| USB programming works but Serial1 fails | Programming USB uses Serial0, GUI uses Serial1 | Choose adapter COM port; verify UART wiring |
| `PING` works but arming blocked | PONG does not verify firmware version or mode | Request V5 handshake and check DISARMED state |
| Motor does not respond | Driver power, enable wires, PWM wiring, selected motor and mode, driver grounds | Test one motor with propellers removed; confirm commanded PWM only |
| E-STOP / FAILSAFE latch | Watchdog expiry, repeated motor test, disconnect, intentional E-STOP | Correct cause, stop, use explicit RESET; never bypass latch |
| Motor turns in wrong direction | Actual mount orientation and sign configuration | Verify low-power direction; adjust `MOTOR_DIR` or test inversion as appropriate |
| No IMU data | SDA20/SCL21, ground, address, module supply | Scan I2C / inspect firmware diagnostics |
| No water temperature | A15, 4.7 kΩ pull-up, sensor supply | Inspect OneWire wiring and telemetry |
| Relay behaves backwards | Relay module active-high vs active-low | Verify de-energized safe state before powering loads |
| RTSP connects but blank video | User/password, profile path, TCP vs UDP, camera network | Try correct stream endpoint; keep credentials out of repo |
| PyInstaller Qt conflict | PyQt6 mixed with PySide6 environment | Recreate clean Python 3.12 venv with PySide6 only |
| GitHub Actions fail | See workflow step logs and source branch | Fix errors locally; workflow checks do not require physical hardware |

**Security:** Never paste SSH private keys, camera passwords, `.env`, access tokens or sensitive device configuration into Issues. Replace exposed credentials if ever committed.
