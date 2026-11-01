# System architecture — ROV Control Station V5

All Mermaid diagrams below are rendered by GitHub and can be edited as text. They describe the implementation and intended wiring, not an independently verified electrical schematic.

## 1. System block diagram

```mermaid
flowchart LR
    PAD[PS4 / USB controller] --> PC[Python PySide6 mission control]
    KEY[Keyboard / motor test buttons] --> PC
    PC <-->|Serial protocol V5 115200 baud| USB[USB to TTL adapter]
    USB <-->|TX1 D18 / RX1 D19| MEGA[Arduino Mega 2560]
    MEGA --> PWM[6 BTS7960 drivers]
    PWM --> MOT[Six thrusters M0 to M5]
    IMU[MPU6050 I2C] --> MEGA
    TMP[DS18B20 OneWire] --> MEGA
    MEGA --> REL[Three relay outputs]
    CAM[RTSP network camera] --> PC
    CAD[Local STL model] --> PC
    PC --> LOG[Local CSV logs]
```

## 2. Serial data and control

```mermaid
sequenceDiagram
    participant GUI as PySide6 GUI
    participant TTL as USB-TTL
    participant MCU as Mega Serial1
    GUI->>TTL: SYS,HELLO,5
    TTL->>MCU: SYS,HELLO,5
    MCU-->>GUI: SYS:HELLO,5,ROV_MEGA6
    GUI->>MCU: SYS,MODE,INDIVIDUAL (while disarmed)
    MCU-->>GUI: SYS:MODE,INDIVIDUAL
    GUI->>MCU: SYS,ARM
    MCU-->>GUI: SYS:STATE,ARMED
    GUI->>MCU: MT,seq,index,power (held motor)
    MCU-->>GUI: MT:ACK,seq,index,power
    GUI->>MCU: MT,seq,-1,0 (release)
    MCU-->>GUI: MT:ACK,seq,-1,0
    GUI->>MCU: SYS,DISARM
```

## 3. Software control modes

```mermaid
flowchart TD
    OFF[DISARMED] -->|Select DRIVE while disarmed| D[DRIVE mode]
    OFF -->|Select INDIVIDUAL while disarmed| T[INDIVIDUAL / motor test mode]
    D -->|Handshake and valid ARM| AD[ARMED: drive packets]
    T -->|Handshake and valid ARM| AT[ARMED: one motor test]
    AD -->|DISARM| OFF
    AT -->|DISARM| OFF
    AD -->|E-STOP / timeout| L[LATCHED ESTOP / FAILSAFE]
    AT -->|E-STOP / packet watchdog / duration limit| L
    L -->|Resolve cause; RESET; confirm DISARMED| OFF
```

Mode and safety-state transitions are simplified for readability; the firmware source is authoritative. Neither GUI E-STOP nor a firmware timeout replaces a **hardwired motor-power isolation switch**.

## 4. Physical topology

- Arduino Mega **D18 TX1** to adapter RXD (level protection where required), **D19 RX1** from adapter TXD, and common GND.
- PWM and enable connections: see [Engineering guide](ENGINEERING_GUIDE.md#motor-driver-pin-map).
- MPU6050: SDA D20, SCL D21. DS18B20: data A15 with 4.7 kΩ pull-up.
- RTSP camera communicates with PC over the camera's network path, *not* Arduino Serial1.
- 3D STL visualization is illustrative/simulated, not measured pose or subsea navigation.
