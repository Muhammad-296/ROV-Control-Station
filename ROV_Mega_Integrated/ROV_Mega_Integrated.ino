/*
  ROV INTEGRATED V5 - SAFE SERIAL1 + INDIVIDUAL MOTOR CONTROL (UART1 Mega D18/D19)  (+ MPU-6050 IMU + 3 relays + DS18B20 water temperature)
  Arduino MEGA 2560 + 6x BTS7960 + 6 motors (thrusters) + MPU-6050 + 3 relay channels + DS18B20

  COMMUNICATION: Serial1 hardware UART @ 115200 baud
      Arduino Mega TX1 (pin 18) --> USB-TTL adapter RXD (protect 3.3 V-only RXD from 5 V)
      Arduino Mega RX1 (pin 19) <-- USB-TTL adapter TXD
      Arduino Mega GND          --- USB-TTL adapter GND
      USB-TTL terminal settings: 115200 baud, line ending = "Newline"
      Handshake: PING -> PONG (can be requested any time, without resetting Mega)
      The Arduino IDE Serial Monitor on the Mega's USB port (Serial0) will NOT see this data.
      Use the serial COM port belonging to your USB-TTL adapter.
      NOTE: Do NOT connect 5 V or 3.3 V adapter supply to a separately powered Mega.
      NOTE: Relays on 15/16/17; UART1 occupies 18/19, so there is no pin conflict.

  LIBRARIES (Arduino IDE -> Library Manager)
      OneWire          (Paul Stoffregen)
      DallasTemperature (Miles Burton)

  MPU-6050 WIRING (I2C)
      VCC -> 5V (GY-521 board has a regulator)   GND -> GND
      SDA -> Mega pin 20                          SCL -> Mega pin 21
      AD0 -> GND (address 0x68)  or 3.3V/VCC (0x69, then set MPU_ADDR below)
      Keep the I2C wires short / twisted, away from motor wires.

  DS18B20 WIRING (waterproof probe)
      Red    -> 5V
      Black  -> GND
      Yellow -> A15 (data)
      4.7k pull-up resistor between the DATA wire and 5V.

  RELAY WIRING (signal pins)
      Pin 17 -> Solenoid Valve relay   (relay 0)
      Pin 16 -> Light relay            (relay 1)
      Pin 15 -> Alert relay            (relay 2)
      Relay board VCC/GND -> 5V/GND (use a separate supply for heavy loads, common GND).
      Most relay boards are ACTIVE-LOW (IN pin LOW = relay ON). If yours is, set
      RELAY_ACTIVE_HIGH = false below. All relays are OFF at power-up.

  THRUSTER LAYOUT (top view, front = up)
      M0 (FL)  \        /  M1 (FR)       horizontal thrusters at 45 deg
                 [ ROV ]
      M2 (RL)  /        \  M3 (RR)
      M4 = vertical LEFT      M5 = vertical RIGHT

  V5 STRICT HANDSHAKE / ARMING (must use matching GUI):
      PING            -> PONG (link heartbeat, NEVER feeds motor watchdog)
      SYS,HELLO,5     -> SYS:HELLO,5,ROV_MEGA6
      SYS,STATE       -> SYS:STATE,DISARMED|ARMED|ESTOP|FAILSAFE, plus SYS:MODE
      SYS,MODE,DRIVE / SYS,MODE,INDIVIDUAL  -> mode switch (only DISARMED)
      SYS:MODE,DRIVE / SYS:MODE,INDIVIDUAL  -> firmware mode confirmation
      SYS,ARM         -> authorizes drive commands ONLY from DISARMED
      SYS,DISARM      -> stops outputs, returns DISARMED (does NOT clear latches)
      SYS,ESTOP       -> immediate stop; latched until SYS,RESET
      SYS,RESET       -> clears ESTOP or FAILSAFE to DISARMED, never arms
      X               -> immediate stop and DISARM (no latch)
      F 500..3000     -> independent motor watchdog; cannot be disabled
      Non-driving commands / PING cannot keep motors alive.
      Relay0 (valve) can only be turned ON in ARMED DRIVE state.
      All O relay commands are rejected in INDIVIDUAL mode.
      All relay loads are OFF after E-STOP, FAILSAFE or lost link.
      INDIVIDUAL test timeout 225 ms; continuous running limit 8 seconds.

  COMMANDS (type then press Enter). Optional power 0-100 after the letter.
      W [p]  forward          S [p]  backward
      A [p]  strafe left      D [p]  strafe right
      Q [p]  yaw left         E [p]  yaw right
      U [p]  up (ascend)      J [p]  down (descend)
      R [p]  roll right       L [p]  roll left
      X      STOP all motors
      + / -  increase / decrease default power by 10%
      P n    set default power to n %
      MT,seq,index,power  index=0..5, power=-60..60, INDIVIDUAL only
      NOTE: legacy M i v and MV are rejected (use MT instead).
      V5 bench test: hold 1..6 to send one MT motor command every 50 ms; release sends MT stop immediately.
      O r v  relay r (0=Solenoid valve, 1=Light, 2=Alert) v = 1 on / 0 off
             Explicit O r v required; no toggle to avoid accidental relay activation
      C ms   smooth damped ramp time (100-5000 ms)
      V s w y h r  proportional drive (-100..100 each): surge sway yaw heave roll  (used by GUI)
      F ms   motor failsafe timeout 500..3000 ms (cannot be disabled)
      G 1/0  motor telemetry on/off (prints PWM:a,b,c,d,e,f every 100 ms)
      I 1/0  IMU telemetry on/off   (prints IMU:roll,pitch,yaw,ax,ay,az,gx,gy,gz,temp every 50 ms)
      Z      zero yaw (set current heading = 0)
      K      calibrate gyro bias (keep ROV still, motors stopped, ~1 s)
      PING   reply PONG (hardware UART1 communication test)
      T      print status
      H      help
  Examples: SYS,MODE,INDIVIDUAL / SYS,ARM / MT,1,0,20 / MT,2,-1,0

  Relay state is reported as  RLY:valve,light,alert  (1 = on) after every change.
  Water temperature is reported as  TMP:23.44  (deg C) about once per second, always on.

  IMU units: angles = deg, accel = g, gyro = deg/s, temp = deg C
  Yaw comes from gyro integration only (MPU-6050 has no magnetometer) so it slowly drifts.

  Mode cannot change while ARMED. Every new mode is DISARMED initially.
  If a motor spins the wrong way, flip its sign in MOTOR_DIR[].
  If the IMU is mounted rotated, flip signs / change axes in IMU_SIGN[].
*/

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const uint8_t MOTOR_COUNT = 6;

// Hardware PWM output pins, Arduino MEGA 2560
const uint8_t RPWM[6] = {2, 4, 6, 8, 10, 12};
const uint8_t LPWM[6] = {3, 5, 7, 9, 11, 13};

// All 12 enable pins are independently wired odd digital pins
const uint8_t R_EN[6] = {23, 27, 31, 35, 39, 43};
const uint8_t L_EN[6] = {25, 29, 33, 37, 41, 45};

// ---------------- Relays ----------------
const uint8_t RELAY_COUNT = 3;
const uint8_t RELAY_PIN[RELAY_COUNT] = {17, 16, 15};   // 0 = Solenoid Valve, 1 = Light, 2 = Alert
const char*   RELAY_NAME[RELAY_COUNT] = {"Solenoid valve", "Light", "Alert"};
const bool    RELAY_ACTIVE_HIGH = true;                // false for active-LOW relay boards
bool          relayState[RELAY_COUNT] = {false, false, false};

// ---------------- Settings ----------------
const long    BAUD_RATE        = 115200;
const int8_t  MOTOR_DIR[6]     = {1, 1, 1, 1, 1, 1};  // set -1 to reverse a motor
const uint8_t MAX_PWM          = 255;                 // global PWM limit
uint16_t scurveTimeMs          = 1200;                // time for a FULL-scale change (0 -> 255), change with "C ms"
const uint16_t MIN_RAMP_MS     = 150;                 // shortest S-curve for small changes
const uint8_t  UPDATE_INTERVAL = 10;                  // ms between output updates
const uint8_t DEADBAND_PWM     = 0;                   // min PWM where motor starts (0 = off)
unsigned long cmdTimeout = 1000;              // 500..3000 ms; cannot be disabled
const unsigned long LINK_TIMEOUT_MS = 3500;  // separately releases relays on lost PING
const unsigned long STATE_INTERVAL_MS = 500;
// Bench-control safety limits: independent of the drive watchdog and PING.
const unsigned long TEST_PACKET_TIMEOUT_MS = 225; // 50 ms refresh, latch after missing traffic
const unsigned long MAX_CONTINUOUS_TEST_MS = 8000; // release between test bursts
unsigned long lastTestPacketMs = 0;
unsigned long motorTestStartedMs = 0;
int8_t activeTestMotor = -1;
uint32_t testAckCount = 0;
enum SafetyState : uint8_t { DISARMED, ARMED, ESTOP, FAILSAFE };
SafetyState safetyState = DISARMED;
enum ControlMode : uint8_t { DRIVE, INDIVIDUAL };
ControlMode controlMode = DRIVE; // safe default on every boot
unsigned long lastLinkTime = 0;
unsigned long lastStateSend = 0;

// ---------------- IMU settings ----------------
const uint8_t  MPU_ADDR        = 0x68;    // 0x69 if AD0 is high
const int8_t   IMU_SIGN[3]     = {1, 1, 1};  // flip an axis (x, y, z) of accel AND gyro if mounted differently
const uint8_t  IMU_INTERVAL    = 10;      // ms between sensor reads (100 Hz filter)
const uint8_t  IMU_SEND_MS     = 50;      // ms between IMU telemetry lines (20 Hz)
const float    FILTER_ALPHA    = 0.98f;   // complementary filter: gyro weight (higher = trust gyro more)
const float    YAW_DEADBAND    = 0.10f;   // deg/s: ignore tiny yaw rates to reduce drift
const float    GYRO_LSB        = 65.5f;   // +-500 deg/s range
const float    ACC_LSB         = 16384.0f;// +-2 g range
// ------------------------------------------

// Motion command in range -100..100 for each axis
int surge = 0, sway = 0, yaw = 0, heave = 0, rollCmd = 0;

int  defaultPower = 50;               // % used when no number is typed
int  targetPWM[6]  = {0, 0, 0, 0, 0, 0};   // signed -255..255
int  currentPWM[6] = {0, 0, 0, 0, 0, 0};
bool manualMode = false;              // true when one MT motor target is active

// Stable critically-damped smoothing. Continuous joystick updates do not restart ramps.
float filteredPWM[6] = {0, 0, 0, 0, 0, 0};
float velocityPWM[6] = {0, 0, 0, 0, 0, 0};

unsigned long lastCmdTime = 0;
unsigned long lastRamp = 0;
String inputLine = "";
bool telemetryOn = false;
unsigned long lastTelem = 0;

// ---------------- IMU state ----------------
bool  imuOk = false;                  // sensor found and answering
bool  imuStream = false;              // "I 1" turns IMU telemetry on
float imuRoll = 0, imuPitch = 0, imuYaw = 0;       // deg
float imuAx = 0, imuAy = 0, imuAz = 0;             // g
float imuGx = 0, imuGy = 0, imuGz = 0;             // deg/s (bias corrected)
float imuTemp = 0;                                 // deg C
float gyroBias[3] = {0, 0, 0};                     // deg/s
uint8_t       imuFail = 0;
bool          imuFirstSample = true;
unsigned long lastImuUs = 0, lastImuRead = 0, lastImuSend = 0, lastImuTry = 0;

// ---------------- DS18B20 water temperature ----------------
const uint8_t  TEMP_PIN        = A15;     // data pin (4.7k pull-up to 5V)
const uint16_t TEMP_INTERVAL   = 1000;    // ms between measurements
const uint16_t TEMP_CONV_MS    = 800;     // 12-bit conversion needs 750 ms
OneWire           oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);
DeviceAddress     tempAddr;
bool          tempOk = false;
bool          tempPending = false;
float         waterTemp = 0;
uint8_t       tempFail = 0;
unsigned long lastTempReq = 0, lastTempTry = 0;

// ---------------- Relay control ----------------
void sendRelayState() {
  Serial1.print(F("RLY:"));
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    Serial1.print(relayState[i] ? 1 : 0);
    if (i < RELAY_COUNT - 1) Serial1.print(',');
  }
  Serial1.println();
}

void setRelay(uint8_t i, bool on) {
  if (i >= RELAY_COUNT) return;
  // The solenoid must never be energized without motor-control authorization.
  if (i == 0 && on && safetyState != ARMED) return;
  relayState[i] = on;
  digitalWrite(RELAY_PIN[i], (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

void allRelaysOff() {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) setRelay(i, false);
  sendRelayState();
}

const __FlashStringHelper* safetyName() {
  switch (safetyState) {
    case ARMED: return F("ARMED");
    case ESTOP: return F("ESTOP");
    case FAILSAFE: return F("FAILSAFE");
    default: return F("DISARMED");
  }
}

const __FlashStringHelper* modeName() {
  return controlMode == INDIVIDUAL ? F("INDIVIDUAL") : F("DRIVE");
}

void sendMode() {
  Serial1.print(F("SYS:MODE,"));
  Serial1.println(modeName());
}

void sendSafetyState() {
  lastStateSend = millis();
  Serial1.print(F("SYS:STATE,"));
  Serial1.println(safetyName());
}

void initRelays() {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    // write the OFF level first so active-low boards never click on at boot
    digitalWrite(RELAY_PIN[i], RELAY_ACTIVE_HIGH ? LOW : HIGH);
    pinMode(RELAY_PIN[i], OUTPUT);
    relayState[i] = false;
  }
}

// ---------------- MPU-6050 low level ----------------
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadReg(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 1, (int)true) != 1) return false;
  val = Wire.read();
  return true;
}

// raw[0..2] accel, raw[3] temp, raw[4..6] gyro
bool mpuReadRaw(int16_t raw[7]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14, (int)true) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) {
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    raw[i] = (hi << 8) | lo;
  }
  return true;
}

bool imuInit() {
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);      // never hang the motor loop if the bus locks up
#endif
  if (!mpuWrite(0x6B, 0x01)) return false;   // wake up, clock = gyro X PLL
  delay(100);
  uint8_t who = 0;
  if (!mpuReadReg(0x75, who)) return false;
  Serial1.print(F("IMU WHO_AM_I = 0x")); Serial1.println(who, HEX);   // 0x68 genuine (clones may differ)
  mpuWrite(0x1A, 0x03);   // digital low-pass filter ~44 Hz (cuts motor vibration noise)
  mpuWrite(0x1B, 0x08);   // gyro  +-500 deg/s
  mpuWrite(0x1C, 0x00);   // accel +-2 g
  mpuWrite(0x19, 0x09);   // sample rate divider (100 Hz)
  return true;
}

// Average the gyro while stationary -> bias. Blocks ~1 s, so only when motors are stopped.
bool imuCalibrate() {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) if (currentPWM[i] != 0 || targetPWM[i] != 0) return false;
  if (!imuOk) return false;
  float sum[3] = {0, 0, 0};
  uint16_t n = 0;
  int16_t raw[7];
  for (uint16_t k = 0; k < 300; k++) {
    if (mpuReadRaw(raw)) {
      sum[0] += raw[4] * IMU_SIGN[0] / GYRO_LSB;
      sum[1] += raw[5] * IMU_SIGN[1] / GYRO_LSB;
      sum[2] += raw[6] * IMU_SIGN[2] / GYRO_LSB;
      n++;
    }
    delay(3);
  }
  if (n < 100) return false;
  for (uint8_t a = 0; a < 3; a++) gyroBias[a] = sum[a] / n;
  imuYaw = 0;
  imuFirstSample = true;      // re-seed roll/pitch from the accelerometer
  lastImuUs = micros();
  return true;
}

void imuUpdate() {
  unsigned long nowMs = millis();

  // Lost / missing sensor: retry every 2 s
  if (!imuOk) {
    if (nowMs - lastImuTry >= 2000) {
      lastImuTry = nowMs;
      if (imuInit()) {
        imuOk = true; imuFail = 0; imuFirstSample = true; lastImuUs = micros();
        Serial1.println(F("IMU: connected"));
      }
    }
    return;
  }

  if (nowMs - lastImuRead < IMU_INTERVAL) return;
  lastImuRead = nowMs;

  int16_t raw[7];
  if (!mpuReadRaw(raw)) {
    if (++imuFail >= 5) {
      imuOk = false;
      Serial1.println(F("IMU: ERROR no answer on I2C (check wiring)"));
    }
    return;
  }
  imuFail = 0;

  unsigned long nowUs = micros();
  float dt = (nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = IMU_INTERVAL * 1e-3f;

  imuAx = raw[0] * IMU_SIGN[0] / ACC_LSB;
  imuAy = raw[1] * IMU_SIGN[1] / ACC_LSB;
  imuAz = raw[2] * IMU_SIGN[2] / ACC_LSB;
  imuTemp = raw[3] / 340.0f + 36.53f;
  imuGx = raw[4] * IMU_SIGN[0] / GYRO_LSB - gyroBias[0];
  imuGy = raw[5] * IMU_SIGN[1] / GYRO_LSB - gyroBias[1];
  imuGz = raw[6] * IMU_SIGN[2] / GYRO_LSB - gyroBias[2];

  float accRoll  = atan2(imuAy, imuAz) * RAD_TO_DEG;
  float accPitch = atan2(-imuAx, sqrt(imuAy * imuAy + imuAz * imuAz)) * RAD_TO_DEG;

  if (imuFirstSample) {
    imuRoll = accRoll;
    imuPitch = accPitch;
    imuFirstSample = false;
  } else {
    imuRoll  = FILTER_ALPHA * (imuRoll  + imuGx * dt) + (1.0f - FILTER_ALPHA) * accRoll;
    imuPitch = FILTER_ALPHA * (imuPitch + imuGy * dt) + (1.0f - FILTER_ALPHA) * accPitch;
  }

  if (fabs(imuGz) > YAW_DEADBAND) imuYaw += imuGz * dt;
  if (imuYaw > 180.0f)  imuYaw -= 360.0f;
  if (imuYaw < -180.0f) imuYaw += 360.0f;
}

void imuSend() {
  if (!imuStream || !imuOk) return;
  unsigned long now = millis();
  if (now - lastImuSend < IMU_SEND_MS) return;
  lastImuSend = now;
  Serial1.print(F("IMU:"));
  Serial1.print(imuRoll, 1);  Serial1.print(',');
  Serial1.print(imuPitch, 1); Serial1.print(',');
  Serial1.print(imuYaw, 1);   Serial1.print(',');
  Serial1.print(imuAx, 2);    Serial1.print(',');
  Serial1.print(imuAy, 2);    Serial1.print(',');
  Serial1.print(imuAz, 2);    Serial1.print(',');
  Serial1.print(imuGx, 1);    Serial1.print(',');
  Serial1.print(imuGy, 1);    Serial1.print(',');
  Serial1.print(imuGz, 1);    Serial1.print(',');
  Serial1.println(imuTemp, 1);
}

// ---------------- DS18B20 water temperature (non-blocking) ----------------
bool tempInit() {
  tempSensor.begin();
  if (tempSensor.getDeviceCount() == 0) return false;
  if (!tempSensor.getAddress(tempAddr, 0)) return false;
  tempSensor.setResolution(tempAddr, 12);
  tempSensor.setWaitForConversion(false);      // never block the motor loop
  return true;
}

void tempUpdate() {
  unsigned long now = millis();

  // Missing sensor: retry every 3 s
  if (!tempOk) {
    if (now - lastTempTry >= 3000) {
      lastTempTry = now;
      if (tempInit()) {
        tempOk = true; tempFail = 0; tempPending = false;
        Serial1.println(F("TEMP: DS18B20 connected"));
      }
    }
    return;
  }

  if (!tempPending) {
    if (now - lastTempReq >= TEMP_INTERVAL) {
      lastTempReq = now;
      tempSensor.requestTemperaturesByAddress(tempAddr);
      tempPending = true;
    }
  } else if (now - lastTempReq >= TEMP_CONV_MS) {
    tempPending = false;
    float t = tempSensor.getTempC(tempAddr);
    // -127 = disconnected, 85.0 = power-on default (conversion not finished / bad power)
    if (t == DEVICE_DISCONNECTED_C || t == 85.0f) {
      if (++tempFail >= 3) {
        tempOk = false;
        Serial1.println(F("TEMP: ERROR no answer from DS18B20 (check wiring / pull-up)"));
      }
      return;
    }
    tempFail = 0;
    waterTemp = t;
    Serial1.print(F("TMP:"));
    Serial1.println(waterTemp, 2);
  }
}

// ---------------- Motor output ----------------
void setMotorRaw(uint8_t i, int pwm) {
  // ENABLEs are always LOW while disarmed, including before boot completes.
  if (safetyState != ARMED) pwm = 0;
  digitalWrite(R_EN[i], safetyState == ARMED ? HIGH : LOW);
  digitalWrite(L_EN[i], safetyState == ARMED ? HIGH : LOW);
  pwm = constrain(pwm, -(int)MAX_PWM, (int)MAX_PWM);
  if (pwm > 0) {
    analogWrite(LPWM[i], 0);
    analogWrite(RPWM[i], pwm);
  } else if (pwm < 0) {
    analogWrite(RPWM[i], 0);
    analogWrite(LPWM[i], -pwm);
  } else {
    analogWrite(RPWM[i], 0);
    analogWrite(LPWM[i], 0);
  }
}

void stopAll() {
  activeTestMotor = -1;
  motorTestStartedMs = 0;
  surge = sway = yaw = heave = rollCmd = 0;
  manualMode = false;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    targetPWM[i] = 0;
    currentPWM[i] = 0;
    filteredPWM[i] = 0.0f;
    velocityPWM[i] = 0.0f;
    setMotorRaw(i, 0);   // emergency stop is immediate (no ramp)
  }
}

// Convert -100..100 % to signed PWM
int percentToPWM(int pct) {
  pct = constrain(pct, -100, 100);
  if (pct == 0) return 0;
  int mag = map(abs(pct), 1, 100, DEADBAND_PWM, MAX_PWM);
  return pct > 0 ? mag : -mag;
}

// ---------------- Thruster mixing ----------------
void mixThrusters() {
  if (safetyState != ARMED || manualMode) return;

  int m[6];
  // Horizontal (vectored X layout)
  m[0] = surge + sway + yaw;   // Front-Left
  m[1] = surge - sway - yaw;   // Front-Right
  m[2] = surge - sway + yaw;   // Rear-Left
  m[3] = surge + sway - yaw;   // Rear-Right
  // Vertical
  m[4] = heave + rollCmd;      // Vertical Left
  m[5] = heave - rollCmd;      // Vertical Right

  // Normalize if any output exceeds 100 %
  int peak = 100;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) peak = max(peak, abs(m[i]));
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    m[i] = (long)m[i] * 100 / peak;
    targetPWM[i] = percentToPWM(m[i]) * MOTOR_DIR[i];
  }
}

// Damped second-order smoothing: stable even when targets change every 50 ms.
// Near-zero initial acceleration and no oscillation; emergency stops bypass it.
void updateRamp() {
  const unsigned long now = millis();
  unsigned long elapsed = now - lastRamp;
  if (elapsed < UPDATE_INTERVAL) return;
  lastRamp = now;
  const float dt = min(elapsed, 50UL) * 0.001f;
  const float tau = max(0.08f, scurveTimeMs * 0.001f / 4.0f);
  const float omega = 1.0f / tau;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    if (safetyState != ARMED) {
      currentPWM[i] = 0;
      filteredPWM[i] = 0.0f;
      velocityPWM[i] = 0.0f;
    } else if (controlMode == INDIVIDUAL && targetPWM[i] == 0) {
      // Momentary test key released: zero-output bypasses all ramping.
      currentPWM[i] = 0;
      filteredPWM[i] = 0.0f;
      velocityPWM[i] = 0.0f;
    } else {
      const float desired = (float)targetPWM[i];
      const float p0 = filteredPWM[i];
      const float accel = omega * omega * (desired - p0)
                          - 2.0f * omega * velocityPWM[i];
      velocityPWM[i] += accel * dt;
      filteredPWM[i] += velocityPWM[i] * dt;
      // Avoid numerical overshoot and small idle PWM residuals.
      if ((desired >= p0 && filteredPWM[i] > desired) ||
          (desired <= p0 && filteredPWM[i] < desired)) {
        filteredPWM[i] = desired;
        velocityPWM[i] = 0.0f;
      }
      if (fabs(filteredPWM[i] - desired) < 0.4f && fabs(velocityPWM[i]) < 1.0f) {
        filteredPWM[i] = desired;
        velocityPWM[i] = 0.0f;
      }
      currentPWM[i] = (int)lround(filteredPWM[i]);
    }
    setMotorRaw(i, currentPWM[i]);
  }
}

void enterSafeState(SafetyState newState) {
  safetyState = newState;
  stopAll();
  allRelaysOff();
  sendSafetyState();
}

void handleSystemCommand(const String& line) {
  lastLinkTime = millis();
  if (line == "SYS,HELLO,5") {
    Serial1.println(F("SYS:HELLO,5,ROV_MEGA6"));
    sendMode();
    sendSafetyState();
    sendRelayState();
  } else if (line == "SYS,STATE") {
    sendMode();
    sendSafetyState();
    sendRelayState();
  } else if (line == "SYS,MODE,DRIVE" || line == "SYS,MODE,INDIVIDUAL") {
    if (safetyState != DISARMED) {
      Serial1.println(F("SYS:ERR,DISARM_BEFORE_MODE_CHANGE"));
      sendMode();
      return;
    }
    stopAll();
    allRelaysOff(); // motor-test mode never inherits relay outputs
    controlMode = line == "SYS,MODE,INDIVIDUAL" ? INDIVIDUAL : DRIVE;
    sendMode();
    sendSafetyState();
  } else if (line == "SYS,ARM") {
    if (safetyState != DISARMED) {
      Serial1.println(F("SYS:ERR,ARM_BLOCKED_RESET_REQUIRED"));
      sendSafetyState();
      return;
    }
    stopAll();
    safetyState = ARMED;
    lastCmdTime = millis();
    sendSafetyState();
  } else if (line == "SYS,DISARM") {
    if (safetyState == ARMED || safetyState == DISARMED) enterSafeState(DISARMED);
    else sendSafetyState();  // never bypass an E-STOP / FAILSAFE latch
  } else if (line == "SYS,ESTOP") {
    enterSafeState(ESTOP);
  } else if (line == "SYS,RESET") {
    if (safetyState == ARMED) {
      Serial1.println(F("SYS:ERR,DISARM_BEFORE_RESET"));
    } else {
      enterSafeState(DISARMED);  // separate ARM is always needed
    }
  } else {
    Serial1.println(F("SYS:ERR,BAD_SYSTEM_COMMAND"));
  }
}

// ---------------- Help / status ----------------
void printHelp() {
  Serial1.println(F("====== ROV INTEGRATED V5 SERIAL1 ======"));
  Serial1.println(F("SYS,HELLO,5 / SYS,STATE / SYS,ARM / SYS,DISARM"));
  Serial1.println(F("SYS,MODE,DRIVE / SYS,MODE,INDIVIDUAL (DISARMED only)"));
  Serial1.println(F("SYS,ESTOP / SYS,RESET ; X = stop and disarm"));
  Serial1.println(F("W/S  forward/backward   A/D  strafe left/right"));
  Serial1.println(F("Q/E  yaw left/right     U/J  up/down"));
  Serial1.println(F("R/L  roll right/left    X    STOP"));
  Serial1.println(F("+ / -  power +/-10%     P n  set power %"));
  Serial1.println(F("MT,seq,index,power (MOTOR TEST only, max 60%, one motor)"));
  Serial1.println(F("O r v  relay (r: 0=valve 1=light 2=alert, v: 1/0)"));
  Serial1.println(F("C ms  smooth ramp 100..5000 ms"));
  Serial1.println(F("V s w y h r  proportional drive (GUI)"));
  Serial1.println(F("F ms motor timeout 500..3000 ; G 1/0 telemetry"));
  Serial1.println(F("I 1/0 IMU telemetry  Z zero yaw  K calibrate gyro"));
  Serial1.println(F("PING -> PONG    T status    H help"));
  Serial1.println(F("Water temp (DS18B20 on A15) is sent as TMP:xx.xx"));
  Serial1.println(F("Optional power after letter, e.g. 'W 70'"));
  Serial1.println(F("========================================="));
}

void printStatus() {
  sendSafetyState();
  Serial1.print(F("Default power: ")); Serial1.print(defaultPower); Serial1.println(F("%"));
  Serial1.print(F("S-curve time: ")); Serial1.print(scurveTimeMs); Serial1.println(F(" ms"));
  Serial1.print(F("Surge:")); Serial1.print(surge);
  Serial1.print(F(" Sway:")); Serial1.print(sway);
  Serial1.print(F(" Yaw:")); Serial1.print(yaw);
  Serial1.print(F(" Heave:")); Serial1.print(heave);
  Serial1.print(F(" Roll:")); Serial1.println(rollCmd);
  Serial1.print(F("PWM  M0..M5: "));
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    Serial1.print(currentPWM[i]);
    Serial1.print(i < MOTOR_COUNT - 1 ? F(", ") : F("\n"));
  }
  Serial1.print(F("Relays: "));
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    Serial1.print(RELAY_NAME[i]); Serial1.print('=');
    Serial1.print(relayState[i] ? F("ON") : F("OFF"));
    Serial1.print(i < RELAY_COUNT - 1 ? F("  ") : F("\n"));
  }
  Serial1.print(F("IMU: ")); Serial1.print(imuOk ? F("OK") : F("NOT CONNECTED"));
  Serial1.print(F("  stream=")); Serial1.println(imuStream ? F("ON") : F("OFF"));
  if (imuOk) {
    Serial1.print(F("Roll ")); Serial1.print(imuRoll, 1);
    Serial1.print(F("  Pitch ")); Serial1.print(imuPitch, 1);
    Serial1.print(F("  Yaw ")); Serial1.print(imuYaw, 1);
    Serial1.print(F("  Temp ")); Serial1.println(imuTemp, 1);
  }
  Serial1.print(F("Water temp: "));
  if (tempOk) { Serial1.print(waterTemp, 2); Serial1.println(F(" C")); }
  else Serial1.println(F("NOT CONNECTED"));
  sendRelayState();
}

// ---------------- Command parser ----------------
void setMotion(int s, int w, int y, int h, int r) {
  manualMode = false;
  surge = s; sway = w; yaw = y; heave = h; rollCmd = r;
}

void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  line.toUpperCase();

  if (line == "PING") {
    lastLinkTime = millis(); // link keepalive for relays, never motor timeout
    Serial1.println(F("PONG"));
    return;
  }
  if (line.startsWith("SYS,")) {
    handleSystemCommand(line);
    return;
  }

  const char cmd = line.charAt(0);
  String rest = line.substring(1);
  rest.trim();
  lastLinkTime = millis();

  if (cmd == 'X') { // legacy immediate stop; never clears E-STOP or FAILSAFE
    if (safetyState == ARMED || safetyState == DISARMED) enterSafeState(DISARMED);
    else sendSafetyState();
    return;
  }

  // Only a verified firmware ARMED state can move any motor.
  if (cmd == 'V') {
    if (safetyState != ARMED) { Serial1.println(F("SYS:ERR,NOT_ARMED")); return; }
    if (controlMode != DRIVE) { Serial1.println(F("SYS:ERR,WRONG_MODE_DRIVE")); return; }
    int a[5] = {0, 0, 0, 0, 0};
    char buf[64], extra;
    rest.toCharArray(buf, sizeof(buf));
    if (sscanf(buf, "%d %d %d %d %d %c", &a[0], &a[1], &a[2], &a[3], &a[4], &extra) != 5) {
      Serial1.println(F("SYS:ERR,BAD_DRIVE_PACKET"));
      return;
    }
    for (uint8_t i = 0; i < 5; i++) {
      if (a[i] < -100 || a[i] > 100) {
        Serial1.println(F("SYS:ERR,DRIVE_OUT_OF_RANGE"));
        return;
      }
    }
    setMotion(a[0], a[1], a[2], a[3], a[4]);
    lastCmdTime = millis(); // VALID DRIVE PACKETS ONLY
    return;
  }

  // V5 reliable momentary motor-test protocol: MT,<seq>,<index>,<power>.
  // index = -1, power = 0 stops all motors immediately. Each accepted packet
  // receives MT:ACK,seq,index,power. One motor only; no background motion.
  if (line.startsWith("MT,")) {
    if (safetyState != ARMED) { Serial1.println(F("SYS:ERR,NOT_ARMED")); return; }
    if (controlMode != INDIVIDUAL) { Serial1.println(F("SYS:ERR,WRONG_MODE_INDIVIDUAL")); return; }
    unsigned long seq = 0; int idx = -1, power = 0; char extra;
    char buf[64]; line.substring(3).toCharArray(buf, sizeof(buf));
    if (sscanf(buf, "%lu,%d,%d%c", &seq, &idx, &power, &extra) != 3 ||
        seq > 65535UL ||
        idx < -1 || idx >= MOTOR_COUNT || power < -60 || power > 60 ||
        ((idx == -1) != (power == 0))) {
      Serial1.println(F("SYS:ERR,BAD_TEST_PACKET")); return;
    }
    unsigned long stamp = millis();
    if (idx == -1) {
      stopAll(); // immediate PWM=0; no ramp on release
    } else {
      // Switching motors first removes power from all previous motors.
      if (activeTestMotor != idx || activeTestMotor == -1) {
        stopAll();
        motorTestStartedMs = stamp;
      }
      manualMode = true;
      activeTestMotor = idx;
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        targetPWM[i] = i == idx ? percentToPWM(power) * MOTOR_DIR[i] : 0;
        if (i != idx) {
          currentPWM[i] = 0;
          filteredPWM[i] = velocityPWM[i] = 0.0f;
          setMotorRaw(i, 0);
        }
      }
    }
    lastTestPacketMs = stamp;
    lastCmdTime = stamp;
    ++testAckCount;
    Serial1.print(F("MT:ACK,"));
    Serial1.print(seq); Serial1.print(',');
    Serial1.print(idx); Serial1.print(','); Serial1.println(power);
    return;
  }
  if (line.startsWith("MV ")) {
    Serial1.println(F("SYS:ERR,USE_MT_V5"));
    return;
  }
  if (cmd == 'M') { // never allow legacy uncoordinated per-motor overrides
    Serial1.println(F("SYS:ERR,USE_MT_IN_INDIVIDUAL_MODE"));
    return;
  }

  // Single-letter manual movement is for supervised bench testing only.
  if (cmd == 'W' || cmd == 'S' || cmd == 'A' || cmd == 'D' ||
      cmd == 'Q' || cmd == 'E' || cmd == 'U' || cmd == 'J' ||
      cmd == 'R' || cmd == 'L') {
    if (safetyState != ARMED) { Serial1.println(F("SYS:ERR,NOT_ARMED")); return; }
    if (controlMode != DRIVE) { Serial1.println(F("SYS:ERR,WRONG_MODE_DRIVE")); return; }
    int pw = defaultPower;
    if (rest.length() && !isDigit(rest.charAt(0))) {
      Serial1.println(F("SYS:ERR,BAD_POWER")); return;
    }
    if (rest.length()) pw = constrain(rest.toInt(), 0, 100);
    int su=0, sw=0, ya=0, he=0, ro=0;
    switch (cmd) {
      case 'W': su=pw; break; case 'S': su=-pw; break;
      case 'A': sw=-pw; break; case 'D': sw=pw; break;
      case 'Q': ya=-pw; break; case 'E': ya=pw; break;
      case 'U': he=pw; break; case 'J': he=-pw; break;
      case 'R': ro=pw; break; case 'L': ro=-pw; break;
    }
    setMotion(su,sw,ya,he,ro);
    lastCmdTime = millis();
    return;
  }

  switch (cmd) {
    case 'O': { // O <relay 0..2> <0|1>
      if (controlMode == INDIVIDUAL) {
        Serial1.println(F("SYS:ERR,RELAYS_DISABLED_IN_MOTOR_TEST")); break;
      }
      int i, on; char extra;
      char buf[32]; rest.toCharArray(buf, sizeof(buf));
      if (sscanf(buf, "%d %d %c", &i, &on, &extra) != 2 ||
          i < 0 || i >= RELAY_COUNT || (on != 0 && on != 1)) {
        Serial1.println(F("SYS:ERR,BAD_RELAY_COMMAND")); break;
      }
      if (i == 0 && on && safetyState != ARMED) {
        Serial1.println(F("SYS:ERR,VALVE_REQUIRES_ARMED")); break;
      }
      setRelay((uint8_t)i, on != 0);
      sendRelayState();
      break;
    }
    case 'F': {
      if (safetyState == ARMED) { Serial1.println(F("SYS:ERR,DISARM_TO_CONFIGURE")); break; }
      int ms = rest.toInt();
      if (ms < 500 || ms > 3000) { Serial1.println(F("SYS:ERR,FAILSAFE_RANGE_500_3000")); break; }
      cmdTimeout = (unsigned long)ms;
      Serial1.print(F("Failsafe = ")); Serial1.println(cmdTimeout);
      break;
    }
    case 'C': {
      if (safetyState == ARMED) { Serial1.println(F("SYS:ERR,DISARM_TO_CONFIGURE")); break; }
      int ms = rest.toInt();
      if (ms < 100 || ms > 5000) { Serial1.println(F("SYS:ERR,RAMP_RANGE_100_5000")); break; }
      scurveTimeMs = (uint16_t)ms;
      Serial1.print(F("Smooth ramp = ")); Serial1.println(scurveTimeMs);
      break;
    }
    case 'G':
      telemetryOn = rest.toInt() == 1;
      break;
    case 'I':
      imuStream = rest.toInt() == 1;
      if (imuStream && !imuOk) Serial1.println(F("IMU: not connected"));
      break;
    case 'Z':
      if (safetyState == ARMED) Serial1.println(F("SYS:ERR,DISARM_TO_ZERO_IMU"));
      else { imuYaw = 0; Serial1.println(F("Yaw zeroed")); }
      break;
    case 'K':
      if (safetyState == ARMED) { Serial1.println(F("SYS:ERR,DISARM_TO_CALIBRATE")); break; }
      Serial1.println(F("Calibrating gyro - keep still..."));
      Serial1.println(imuCalibrate() ? F("IMU: calibrated") : F("IMU: calibration refused"));
      break;
    case 'P':
      defaultPower = constrain(rest.toInt(), 0, 100);
      break;
    case '+': defaultPower = min(100, defaultPower+10); break;
    case '-': defaultPower = max(0, defaultPower-10); break;
    case 'T': printStatus(); break;
    case 'H':
    case '?': printHelp(); break;
    default: Serial1.println(F("SYS:ERR,UNKNOWN_COMMAND")); break;
  }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial1.begin(BAUD_RATE);

  initRelays();   // all relays OFF at power-up

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    pinMode(RPWM[i], OUTPUT);
    pinMode(LPWM[i], OUTPUT);
    pinMode(R_EN[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);
    analogWrite(RPWM[i], 0);
    analogWrite(LPWM[i], 0);
    digitalWrite(R_EN[i], LOW);    // safe / disabled until SYS,ARM
    digitalWrite(L_EN[i], LOW);
  }

  stopAll();
  controlMode = DRIVE;

  // IMU (motors are stopped, so calibrating here is safe - keep the ROV still at power-up)
  imuOk = imuInit();
  if (imuOk) {
    Serial1.println(F("IMU: MPU-6050 found, calibrating gyro (keep still)..."));
    imuFirstSample = true;
    lastImuUs = micros();
    if (imuCalibrate()) Serial1.println(F("IMU: calibrated"));
  } else {
    Serial1.println(F("IMU: MPU-6050 NOT FOUND (check SDA=20, SCL=21, power, address)"));
  }

  // DS18B20 water temperature
  tempOk = tempInit();
  if (tempOk) Serial1.println(F("TEMP: DS18B20 found"));
  else        Serial1.println(F("TEMP: DS18B20 NOT FOUND (check wiring on A15, 4.7k pull-up)"));

  lastCmdTime = millis();
  lastLinkTime = millis();
  printHelp();
  sendRelayState();
  sendMode();
  sendSafetyState();
  Serial1.println(F("READY"));   // GUI additionally requires SYS:HELLO,5,ROV_MEGA6
}

void loop() {
  // Read bounded serial lines. Overlong input is rejected, never partly executed.
  static bool overflowed = false;
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (overflowed) {
        Serial1.println(F("SYS:ERR,LINE_TOO_LONG"));
      } else if (inputLine.length() > 0) {
        processCommand(inputLine);
      }
      inputLine = "";
      overflowed = false;
    } else if (inputLine.length() < 79 && !overflowed) {
      inputLine += c;
    } else {
      overflowed = true;
    }
  }

  const unsigned long now = millis();
  // Independent fast bench watchdog: stale *nonzero* test output is a fault.
  // Refresh requires a valid MT packet, not PING or unrelated commands.
  if (safetyState == ARMED && controlMode == INDIVIDUAL && activeTestMotor >= 0) {
    if (now - lastTestPacketMs >= TEST_PACKET_TIMEOUT_MS) {
      Serial1.println(F("SYS:ERR,TEST_PACKET_TIMEOUT"));
      enterSafeState(FAILSAFE);
    } else if (now - motorTestStartedMs >= MAX_CONTINUOUS_TEST_MS) {
      Serial1.println(F("SYS:ERR,TEST_DURATION_LIMIT"));
      enterSafeState(FAILSAFE);
    }
  }
  if (safetyState == ARMED && now - lastCmdTime >= cmdTimeout) {
    enterSafeState(FAILSAFE); // latched: a RESET then a NEW ARM is mandatory
  }
  if (now - lastLinkTime >= LINK_TIMEOUT_MS) {
    // Relay safety is independent of the motor watchdog and works disarmed too.
    bool anyRelayOn = false;
    for (uint8_t i=0; i<RELAY_COUNT; i++) if (relayState[i]) anyRelayOn = true;
    if (anyRelayOn) allRelaysOff();
  }

  mixThrusters();
  updateRamp();
  imuUpdate();
  tempUpdate();
  if (now - lastStateSend >= STATE_INTERVAL_MS) sendSafetyState();
  if (telemetryOn && now - lastTelem >= 100) {
    lastTelem = now;
    Serial1.print(F("PWM:"));
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
      Serial1.print(currentPWM[i]);
      if (i < MOTOR_COUNT - 1) Serial1.print(',');
    }
    Serial1.println();
  }
  imuSend();
}
