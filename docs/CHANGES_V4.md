# Integrated V4 — Motor Test Keyboard Mode

- Dedicated MOTOR TEST mode in control selector and dedicated GUI tab.
- Direct momentary `1`–`6` shortcuts: key down = motor ON; key up = all-zero motor packet immediately.
- Six large onscreen hold buttons, one common speed slider (5–60%, 15% default), optional Reverse checkbox.
- Requires mode confirmation and deliberate ENABLE MOTOR TEST (firmware ARM); normal drive remains independent.
- Test-mode keyboard works independently of the ROV Drive keyboard-enable checkbox.
- Normal PS4 drive, W/A/S/D, and relay commands are disabled in test mode. E-STOP/DISARM remain available.
- Zero packet is transmitted immediately on key release, mode tab departure, or focus loss; 50ms neutral keepalive continues while armed.
- Firmware V4 enforces one nonzero test motor, a 60% cap, and immediate PWM zero on release; start acceleration still uses the existing smooth ramp.
- Firmware/GUI version handshake updated to `SYS,HELLO,4` / `SYS:HELLO,4,ROV_MEGA6`.
- Camera, PS4 monitor, 3D viewer, sensor logging, relay controls and existing pins otherwise retained.

All motor tests require propellers removed, a physical cutoff and controlled bench setup. Synthetic validation is not a substitute for hardware testing.
