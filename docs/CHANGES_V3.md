# V3 Change Log — Individual Motor Mode

- Added separate **Individual Motors** GUI tab with six motor rows.
- Each row has a selectable motor, independent 5–100% speed setting (default 20%), and momentary forward/reverse hold buttons.
- Separated keyboard maps: **ROV Drive** uses W/A/S/D/Q/E/U/J/R/L; **Individual** uses **1–6**, **Up/Down**, **Space** (immediate disarm). **Escape** remains latched E-STOP.
- The GUI deliberately commands **one motor at a time** while in Individual mode. This prevents accidental compound thrust during bench checks.
- Added a top-bar **CONTROL MODE** selector and **SYNC MODE** command. The Arduino reports `SYS:MODE,DRIVE` or `SYS:MODE,INDIVIDUAL`; the GUI will not ARM until the mode is confirmed.
- Updated firmware handshake to `SYS,HELLO,3` / `SYS:HELLO,3,ROV_MEGA6` (the V2 GUI/firmware pair is intentionally incompatible with V3).
- Added atomic six-motor command `MV <m0> <m1> <m2> <m3> <m4> <m5>`: a full update always resets any previously active motor.
- Firmware rejects `V` in INDIVIDUAL mode, rejects `MV` in DRIVE mode, rejects legacy `M i v`, and prohibits switching modes while armed or latched.
- Firmware-independent E-STOP, failsafe, relay safety, serial heartbeat, IMU, water temperature, and camera/3D features retained.
- PS4 motion and PS4 ARM are ignored in Individual mode. PS4 DISARM / E-STOP remain available when connected; the PS4 itself is not required for Individual operation.
- Updated test script and README, and updated the optional Windows EXE build name to `ROV_Control_Station_v3.exe`.

## Validation limitations

Passed Python syntax compilation, C++ syntax compilation with mock Arduino APIs, headless GUI state tests, and mock-firmware mode/command state tests. **Not yet compiled in the Arduino IDE or tested with motors, USB-to-TTL hardware, PySide6 on Windows, a PS4, camera, or an actual ROV.** Remove propellers and use an independent physical power cutoff for first real-world tests.
