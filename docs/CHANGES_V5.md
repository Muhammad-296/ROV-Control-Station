# Changes in V5 vs V4

- Dedicated non-modal Motor Test Station window; same verified Serial1 COM connection as dashboard.
- Hold numbers 1–6 / matching buttons to run selected motor; release sends immediate stop. PS4 Drive remains isolated.
- Each motor has a 5–60% power setting and saved direction inversion; common slider applies speed to all.
- New V5 `MT,seq,index,power` command and `MT:ACK,...` response; V4 `MV` rejected.
- Independent 225 ms active-test packet watchdog and 8-second continuous motor test duration limit; faults latch FAILSAFE.
- GUI checks last valid matching motor command acknowledgment and E-STOPs on prolonged missing acknowledgments.
- On focus loss, release keys; on closing motor test window, stop and disarm; mode switching requires DISARMED.
- Test event CSV export; guided manually confirmed PASS/FAIL for M0–M5; telemetry staleness indication.
- Retains 6-BTS7960 wiring, MPU6050, DS18B20, relay logic, PS4 drive, RTSP camera, STL viewer, logs.
- Clean v5 PyInstaller script excludes other Qt bindings to avoid the PySide6/PyQt6 packaging conflict.

Validation: Python `py_compile` and synthetic GUI test passed; mock C++ compiler/state/watchdog tests passed. Arduino IDE compile, PySide6 GUI runtime and physical hardware not verified in this environment.
