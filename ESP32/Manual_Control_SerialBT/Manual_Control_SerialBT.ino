// #include "BluetoothSerial.h"

// BluetoothSerial SerialBT;

// void setup() {
//   Serial.begin(115200);
//   SerialBT.begin("ESP32_BT");   // Name shown when pairing
//   Serial.println("Bluetooth started. Waiting for data...");
// }

// void loop() {
//   if (SerialBT.available()) {
//     char c = SerialBT.read();   // Read 1 character
//     Serial.print("Received via BT: ");
//     Serial.println(c);
//   }
// }


#include <ESP32Servo.h>
#include <math.h>
#include "BluetoothSerial.h"

// =========================
// Config
// =========================
static const int   kNumServos = 5;
static const int   kServoPins[kNumServos] = {13, 12, 14, 27, 26};
static const float kPhaseShift = TWO_PI / kNumServos;   // 2π/5
static const int   kUpdateDelayMs = 20;

// Servo timing (typical)
static const int kMinUs = 500;
static const int kMaxUs = 2400;

// Motion params
static const float kFrequencyHz = 0.5f;   // traveling wave frequency
static const float kCenterDeg   = 90.0f;  // neutral position

// Turning and amplitude smoothing (slew rate limits)
static const float kTurnSlewDegPerSec = 30.0f;
static const float kAmpSlewDegPerSec  = 60.0f;

// Amplitude limits
static const float kAmpMin = 0.0f;
static const float kAmpMax = 60.0f;

// Turning targets
static const float kTurnLeftOffsetDeg  = +15.0f;
static const float kTurnRightOffsetDeg = -15.0f;

// Device name shown in Windows pairing
static const char* kBtName = "ESP32_Snake";


// =========================
// Globals
// =========================
BluetoothSerial SerialBT;
Servo servos[kNumServos];

// Motion state
static bool  motionEnabled = false;

// Targets set by commands
static float targetAmplitudeDeg = 30.0f;
static float targetTurnOffsetDeg = 0.0f;

// Smoothed (applied) values
static float currentAmplitudeDeg = 30.0f;
static float currentTurnOffsetDeg = 0.0f;

// Timekeeping
static unsigned long startTimeMs = 0;
static unsigned long lastLoopMs  = 0;


// =========================
// Helpers
// =========================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline int clampAngleToServo(float deg) {
  deg = clampf(deg, 0.0f, 180.0f);
  return (int)(deg + 0.5f);
}

static void logBoth(const char* msg) {
  Serial.println(msg);
  SerialBT.println(msg);
}

// Slew-limit `current` toward `target` by maxStep
static float slewToward(float current, float target, float maxStep) {
  float diff = target - current;
  if (diff >  maxStep) diff =  maxStep;
  if (diff < -maxStep) diff = -maxStep;
  return current + diff;
}

static void calibrateToCenter() {
  for (int i = 0; i < kNumServos; i++) {
    servos[i].write((int)kCenterDeg);
  }
}

// =========================
// Command Handling
// =========================
// Commands supported (same as your original logic):
//  c/C : calibrate + STOP (freeze at 90)
//  l/L : turn left (smooth)
//  r/R : turn right (smooth)
//  s/S : straighten (turn->0) + RESUME motion
//  0..9: amplitude = digit*5 deg
//  +   : amplitude += 5
//  -   : amplitude -= 5
static void processCommand(char c) {
  if (c == 'c' || c == 'C') {
    targetTurnOffsetDeg = 0.0f;
    currentTurnOffsetDeg = 0.0f;

    targetAmplitudeDeg = 0.0f;
    currentAmplitudeDeg = 0.0f;

    calibrateToCenter();
    motionEnabled = false;
    logBoth("Calibrate to center, STOP");

  } else if (c == 'l' || c == 'L') {
    targetTurnOffsetDeg = kTurnLeftOffsetDeg;
    logBoth("Turn LEFT (smooth)");

  } else if (c == 'r' || c == 'R') {
    targetTurnOffsetDeg = kTurnRightOffsetDeg;
    logBoth("Turn RIGHT (smooth)");

  } else if (c == 's' || c == 'S') {
    targetTurnOffsetDeg = 0.0f;
    motionEnabled = true;
    logBoth("Straight & RESUME motion (smooth)");

  } else if (c >= '0' && c <= '9') {
    int d = c - '0';
    targetAmplitudeDeg = clampf(d * 5.0f, kAmpMin, kAmpMax);

    Serial.print("Set amplitude target to: ");
    Serial.println(targetAmplitudeDeg);
    SerialBT.print("Set amplitude target to: ");
    SerialBT.println(targetAmplitudeDeg);

  } else if (c == '+') {
    targetAmplitudeDeg = clampf(targetAmplitudeDeg + 5.0f, kAmpMin, kAmpMax);

    Serial.print("Amplitude target: ");
    Serial.println(targetAmplitudeDeg);
    SerialBT.print("Amplitude target: ");
    SerialBT.println(targetAmplitudeDeg);

  } else if (c == '-') {
    targetAmplitudeDeg = clampf(targetAmplitudeDeg - 5.0f, kAmpMin, kAmpMax);

    Serial.print("Amplitude target: ");
    Serial.println(targetAmplitudeDeg);
    SerialBT.print("Amplitude target: ");
    SerialBT.println(targetAmplitudeDeg);
  }
}

static void handleBluetooth() {
  while (SerialBT.available() > 0) {
    char c = (char)SerialBT.read();
    processCommand(c);

    // Optional debug echo:
    // Serial.print("RX: "); Serial.println(c);
  }
}


// =========================
// Motion Update
// =========================
static void updateMotion(float dtSec, unsigned long nowMs) {
  // Smooth values every loop while moving
  const float maxTurnStep = kTurnSlewDegPerSec * dtSec;
  const float maxAmpStep  = kAmpSlewDegPerSec  * dtSec;

  currentTurnOffsetDeg = slewToward(currentTurnOffsetDeg, targetTurnOffsetDeg, maxTurnStep);
  currentAmplitudeDeg  = slewToward(currentAmplitudeDeg,  targetAmplitudeDeg,  maxAmpStep);

  float tSec  = (nowMs - startTimeMs) / 1000.0f;
  float omega = TWO_PI * kFrequencyHz;

  for (int i = 0; i < kNumServos; i++) {
    float phase = i * kPhaseShift;
    float angle = (kCenterDeg + currentTurnOffsetDeg)
                + currentAmplitudeDeg * sinf(omega * tSec + phase);

    servos[i].write(clampAngleToServo(angle));
  }
}


// =========================
// Setup / Loop
// =========================
void setup() {
  Serial.begin(115200);
  delay(200);

  if (!SerialBT.begin(kBtName)) {
    Serial.println("Bluetooth failed to start!");
  } else {
    Serial.print("Bluetooth started. Pair to: ");
    Serial.println(kBtName);
  }

  // PWM timers for ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach servos
  for (int i = 0; i < kNumServos; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(kServoPins[i], kMinUs, kMaxUs);
  }

  startTimeMs = millis();
  lastLoopMs  = startTimeMs;

  calibrateToCenter();
  logBoth("Ready. Press 's' to start motion. 'c' to calibrate/stop.");
}

void loop() {
  handleBluetooth();

  unsigned long nowMs = millis();
  float dtSec = (nowMs - lastLoopMs) / 1000.0f;
  lastLoopMs = nowMs;

  if (motionEnabled) {
    updateMotion(dtSec, nowMs);
  }

  delay(kUpdateDelayMs);
}
