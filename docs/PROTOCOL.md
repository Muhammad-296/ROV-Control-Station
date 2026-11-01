# Serial protocol reference — V5

Transport: **Arduino Mega Serial1, 115200 baud**, newline-terminated ASCII commands. The desktop GUI must use the **matching V5 firmware**, not V3/V4.

| PC → firmware | Firmware → PC / meaning |
|:--|:--|
| `SYS,HELLO,5` | `SYS:HELLO,5,ROV_MEGA6` handshake |
| `PING` | `PONG` link check only |
| `SYS,MODE,DRIVE` | `SYS:MODE,DRIVE` while disarmed |
| `SYS,MODE,INDIVIDUAL` | `SYS:MODE,INDIVIDUAL` while disarmed |
| `SYS,ARM` | `SYS:STATE,ARMED` if authorized |
| `SYS,DISARM` | `SYS:STATE,DISARMED` |
| `SYS,ESTOP` | Latched `SYS:STATE,ESTOP` |
| `SYS,RESET` | Attempt to clear latch; does not arm |
| `V surge sway yaw heave roll` | Drive mode only |
| `MT,1,0,15` | Test M0 at +15%; `MT:ACK,1,0,15` |
| `MT,2,3,-20` | Test M3 at −20%; `MT:ACK,2,3,-20` |
| `MT,3,-1,0` | Stop all test motors; acknowledgment |
| `G 1`, `I 1` | Firmware telemetry requests (PWM / IMU) |

Additional inbound lines include `PWM:`, `IMU:`, `TMP:` and `RLY:`. `MT` sequence is 0..65535, motor index 0..5, signed power −60..60, with `-1,0` reserved for stop. Only one test motor may be active. Acknowledgment means the firmware accepted a command, **not** that a motor physically rotated. An active test packet must be refreshed within **225 ms**; continuous motor-test operation is limited to **8 seconds** before failsafe. Do not operate multiple applications on the same serial connection.

For protocol implementation details and edge cases, the `.ino` source is authoritative. See [Architecture](ARCHITECTURE.md).
