/*
  ESP32 Snake Robot — Unified Bluetooth Control + IMU/Optical-Flow Telemetry (12 bytes)

  ✅ Same Bluetooth SPP link does BOTH:
    1) RX control commands from PC (single characters):  c l r s 0..9 + -
    2) TX live telemetry to PC (binary): 3 float32 values = 12 bytes total
        [direction_deg][xw_m][yw_m]  each is 4 bytes (IEEE754 float, little-endian)

  ✅ Keeps YOUR position math:
    - yaw0 capture
    - theta = wrap(yaw - yaw0)
    - rotation matrix
    - integrate xw_counts/yw_counts
    - xw_m = xw_counts * meters_per_count, same placeholder

  Notes:
  - On PC, read telemetry using ser.read(12) and struct.unpack("<fff", data)
  - Avoid printing lots of text over Bluetooth while also sending binary (we only print to USB Serial).
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "BluetoothSerial.h"
#include "Bitcraze_PMW3901.h"
#include <Adafruit_BNO08x.h>

#include <ESP32Servo.h>

// =====================================================
// Pins
// =====================================================
// IMU (BNO08x) over SPI
#define BNO08X_CS    15
#define BNO08X_INT   22
#define BNO08X_RESET 4

// Optical flow (PMW3901) over SPI
#define FLOW_CS      5

// Shared SPI pins (ESP32 DOIT)
static const int PIN_SCK  = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;

// Servos
static const int   kNumServos = 5;
static const int   kServoPins[kNumServos] = {13, 12, 14, 27, 26};

// =====================================================
// Bluetooth
// =====================================================
static const char* kBtName = "ESP32_Snake";
BluetoothSerial SerialBT;

// =====================================================
// Sensors
// =====================================================
Bitcraze_PMW3901 flow(FLOW_CS);
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Use same report as your final code
static const sh2_SensorId_t kReportType = SH2_GAME_ROTATION_VECTOR;

// =====================================================
// Motion (servos)
// =====================================================
static const float kFrequencyHz = 0.5f;
static const float kCenterDeg   = 90.0f;
static const float kPhaseShift  = TWO_PI / kNumServos;

static const float kTurnSlewDegPerSec = 30.0f;
static const float kAmpSlewDegPerSec  = 60.0f;

static const float kAmpMin = 0.0f;
static const float kAmpMax = 60.0f;

static const float kTurnLeftOffsetDeg  = +15.0f;
static const float kTurnRightOffsetDeg = -15.0f;

static const int kUpdateDelayMs = 20;
static const int kServoMinUs = 500;
static const int kServoMaxUs = 2400;

Servo servos[kNumServos];

static bool  motionEnabled = false;
static float targetAmplitudeDeg  = 30.0f;
static float currentAmplitudeDeg = 30.0f;
static float targetTurnOffsetDeg  = 0.0f;
static float currentTurnOffsetDeg = 0.0f;

static unsigned long startTimeMs = 0;
static unsigned long lastLoopMs  = 0;

// =====================================================
// Position (world frame) + yaw handling (YOUR math)
// =====================================================
static float xw_counts = 0.0f;
static float yw_counts = 0.0f;

static bool  yawInitialized = false;
static float yaw0_deg   = 0.0f;   // captured once
static float lastYawDeg = 0.0f;   // latest yaw reading
static float theta_deg  = 0.0f;   // wrapped relative yaw
static float theta_rad  = 0.0f;

// Placeholder scale (same as your code)
static const float meters_per_count = 1.0f / 60.0f;

// =====================================================
// Telemetry timing
// =====================================================
static const uint32_t kTelemetryPeriodMs = 100; // 10 Hz
static uint32_t lastTelemetryMs = 0;

// =====================================================
// Helpers
// =====================================================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float wrapDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static inline float slewToward(float current, float target, float maxStep) {
  float diff = target - current;
  if (diff >  maxStep) diff =  maxStep;
  if (diff < -maxStep) diff = -maxStep;
  return current + diff;
}

static inline int clampAngleToServo(float deg) {
  deg = clampf(deg, 0.0f, 180.0f);
  return (int)(deg + 0.5f);
}

static void calibrateToCenter() {
  for (int i = 0; i < kNumServos; i++) {
    servos[i].write((int)kCenterDeg);
  }
}

// Write one float32 to BT as 4 bytes (little-endian on ESP32)
static void writeFloatLE(BluetoothSerial& bt, float v) {
  uint8_t b[4];
  memcpy(b, &v, 4);
  bt.write(b, 4);
}

static void setReports() {
  if (!bno08x.enableReport(kReportType)) {
    Serial.println("BNO08x: Could not enable GAME ROTATION VECTOR");
  }
}

// Yaw from GAME rotation vector quaternion (your exact formula)
static float yawFromGameQuatDeg(const sh2_SensorValue_t& sv) {
  const float qr = sv.un.gameRotationVector.real;
  const float qi = sv.un.gameRotationVector.i;
  const float qj = sv.un.gameRotationVector.j;
  const float qk = sv.un.gameRotationVector.k;

  const float sqr = qr*qr, sqi = qi*qi, sqj = qj*qj, sqk = qk*qk;
  float yaw = atan2f(2.0f * (qi*qj + qk*qr), (sqi - sqj - sqk + sqr));
  return yaw * (180.0f / PI);
}

// =====================================================
// Bluetooth RX (control)
// =====================================================
// Commands:
//  c/C : calibrate + STOP
//  l/L : turn left (targetTurnOffset = +15)
//  r/R : turn right (targetTurnOffset = -15)
//  s/S : straighten + RESUME motion
//  0..9: amplitude = digit*5 deg
//  + / -: amplitude +/- 5
static void processCommand(char c) {
  if (c == 'c' || c == 'C') {
    targetTurnOffsetDeg  = 0.0f;
    currentTurnOffsetDeg = 0.0f;
    targetAmplitudeDeg   = 0.0f;
    currentAmplitudeDeg  = 0.0f;
    calibrateToCenter();
    motionEnabled = false;

  } else if (c == 'l' || c == 'L') {
    targetTurnOffsetDeg = kTurnLeftOffsetDeg;

  } else if (c == 'r' || c == 'R') {
    targetTurnOffsetDeg = kTurnRightOffsetDeg;

  } else if (c == 's' || c == 'S') {
    targetTurnOffsetDeg = 0.0f;
    motionEnabled = true;

  } else if (c >= '0' && c <= '9') {
    int d = c - '0';
    targetAmplitudeDeg = clampf(d * 5.0f, kAmpMin, kAmpMax);

  } else if (c == '+') {
    targetAmplitudeDeg = clampf(targetAmplitudeDeg + 5.0f, kAmpMin, kAmpMax);

  } else if (c == '-') {
    targetAmplitudeDeg = clampf(targetAmplitudeDeg - 5.0f, kAmpMin, kAmpMax);
  }
}

static void handleBluetoothRx() {
  while (SerialBT.available() > 0) {
    char c = (char)SerialBT.read();
    processCommand(c);
  }
}

// =====================================================
// Sensors + position update (YOUR math preserved)
// =====================================================
static void updateSensorsAndPosition() {
  // ---- Optical flow increment (body frame) ----
  int16_t dx = 0, dy = 0;
  flow.readMotionCount(&dx, &dy);

  // ---- IMU yaw update (degrees) using GAME ROTATION VECTOR ----
  if (bno08x.wasReset()) {
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == kReportType) {
      float yawDeg = yawFromGameQuatDeg(sensorValue);
      lastYawDeg = yawDeg;

      // Capture initial yaw once
      if (!yawInitialized) {
        yaw0_deg = lastYawDeg;
        yawInitialized = true;
      }
    }
  }

  // If no yaw yet, skip rotation/integration to avoid junk
  if (!yawInitialized) return;

  // ---- Relative heading theta ----
  theta_deg = wrapDeg(lastYawDeg - yaw0_deg);
  theta_rad = theta_deg * (PI / 180.0f);

  // ---- Rotation matrix from theta ----
  const float c = cosf(theta_rad);
  const float s = sinf(theta_rad);

  // Rotate increment into world frame and integrate
  const float dXw = c * (float)dx - s * (float)dy;
  const float dYw = s * (float)dx + c * (float)dy;

  xw_counts += dXw;
  yw_counts += dYw;
}

// =====================================================
// Motion update (servos)
// =====================================================
static void updateMotion(float dtSec, unsigned long nowMs) {
  // Smooth turn + amplitude (slew-limit)
  const float maxTurnStep = kTurnSlewDegPerSec * dtSec;
  const float maxAmpStep  = kAmpSlewDegPerSec  * dtSec;

  currentTurnOffsetDeg = slewToward(currentTurnOffsetDeg, targetTurnOffsetDeg, maxTurnStep);
  currentAmplitudeDeg  = slewToward(currentAmplitudeDeg,  targetAmplitudeDeg,  maxAmpStep);

  // Traveling wave
  const float tSec  = (nowMs - startTimeMs) / 1000.0f;
  const float omega = TWO_PI * kFrequencyHz;

  for (int i = 0; i < kNumServos; i++) {
    const float phase = i * kPhaseShift;
    const float angle = (kCenterDeg + currentTurnOffsetDeg)
                      + currentAmplitudeDeg * sinf(omega * tSec + phase);

    servos[i].write(clampAngleToServo(angle));
  }
}

// =====================================================
// Telemetry TX (12 bytes total): direction + x + y (float32 each)
// =====================================================
static void sendTelemetry12B() {
  if (!SerialBT.hasClient()) return;
  if (!yawInitialized) return;

  // Keep YOUR conversion from counts->meters
  const float xw_m = xw_counts * meters_per_count;
  const float yw_m = yw_counts * meters_per_count;

  // Direction: your wrapped relative heading in degrees
  const float direction_deg = theta_deg;

  // Exactly 12 bytes: 3 floats
  writeFloatLE(SerialBT, direction_deg);
  writeFloatLE(SerialBT, xw_m);
  writeFloatLE(SerialBT, yw_m);
}

// =====================================================
// Setup / Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // ---- Init shared SPI bus ----
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  // CS lines idle HIGH
  pinMode(FLOW_CS, OUTPUT);
  pinMode(BNO08X_CS, OUTPUT);
  digitalWrite(FLOW_CS, HIGH);
  digitalWrite(BNO08X_CS, HIGH);
  delay(50);

  // ---- Init PMW3901 ----
  if (!flow.begin()) {
    Serial.println("PMW3901 init FAILED");
    while (1) delay(100);
  }
  Serial.println("PMW3901 init OK");

  // ---- Init BNO08x over SPI ----
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("BNO08x init FAILED");
    while (1) delay(100);
  }
  Serial.println("BNO08x init OK");
  setReports();

  // ---- Bluetooth ----
  if (!SerialBT.begin(kBtName)) {
    Serial.println("Bluetooth failed to start!");
  } else {
    Serial.print("Bluetooth started. Pair to: ");
    Serial.println(kBtName);
  }

  // ---- Servos ----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < kNumServos; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(kServoPins[i], kServoMinUs, kServoMaxUs);
  }

  calibrateToCenter();

  startTimeMs = millis();
  lastLoopMs  = startTimeMs;
  lastTelemetryMs = startTimeMs;

  Serial.println("Ready.");
  Serial.println("Waiting for initial yaw (first IMU sample)...");
  Serial.println("Commands over BT: c l r s 0..9 + -");
  Serial.println("Telemetry over BT: 12 bytes per frame (dir_deg, xw_m, yw_m) as float32 LE");
}

void loop() {
  // 1) Receive control commands (fast, non-blocking)
  handleBluetoothRx();

  // 2) Update sensors + integrate position (your math)
  updateSensorsAndPosition();

  // 3) Motion timing
  const unsigned long nowMs = millis();
  const float dtSec = (nowMs - lastLoopMs) / 1000.0f;
  lastLoopMs = nowMs;

  if (motionEnabled) {
    updateMotion(dtSec, nowMs);
  }

  // 4) Telemetry timing (fixed rate)
  if (nowMs - lastTelemetryMs >= kTelemetryPeriodMs) {
    lastTelemetryMs = nowMs;
    sendTelemetry12B();
  }

  delay(kUpdateDelayMs);
}
