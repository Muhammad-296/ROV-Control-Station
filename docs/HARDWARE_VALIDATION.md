# Hardware verification record

**Project status:** Hardware testing has been **reported by the project owner**. This file is a template for repeatable results; no individual results, dates, underwater ratings or compliance claims have been independently supplied. A blank entry means **not documented**, not failed or passed.

## Test environment

| Field | Owner entry |
|:--|:--|
| Test date and operator | _To be recorded_ |
| Firmware and GUI revisions / commit | _To be recorded_ |
| Power supply and maximum current | _To be recorded_ |
| Motor / propeller configuration | _To be recorded_ |
| Hardware E-STOP and isolation method | _To be recorded_ |
| Location: dry bench / tank / open water | _To be recorded_ |
| Evidence (logs, photos, video) | _To be linked_ |

## Verification checklist (fill in actual results)

| Test | Expected outcome | Result | Evidence |
|:--|:--|:--|:--|
| Correct Mega pin mapping M0–M5 | Each physical motor matches documented connector | Not recorded | — |
| Individual low-power motor test | Only selected motor commands PWM | Not recorded | — |
| Key release | Zero PWM command is sent promptly | Not recorded | — |
| Mode separation | Motor-test and normal drive packets remain isolated | Not recorded | — |
| Firmware handshake | GUI receives V5 identity and mode | Not recorded | — |
| Link loss / failsafe | MCU transitions to safe commanded state | Not recorded | — |
| Latched E-STOP and reset | Explicit reset is required before rearming | Not recorded | — |
| MPU6050 and DS18B20 | Plausible readings observed | Not recorded | — |
| Relay off-state / polarity | Loads de-energize as required | Not recorded | — |
| Physical E-STOP power cutoff | Motor supply can be isolated | Not recorded | — |
| RTSP feed | Stream opens with valid runtime credentials | Not recorded | — |
| Camera / enclosure underwater validation | Documented waterproofing and strain relief | Not recorded | — |

## Review note

Do initial commissioning with **propellers removed**, a secured assembly and independently reachable motor-power isolation. Software E-STOP and `PWM:0` do not guarantee instantaneous physical rotation stop. Record failed cases as well as successful cases. Hardware-tested is **not** the same as safety-certified or underwater-rated.
