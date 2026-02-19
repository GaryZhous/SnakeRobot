/*
  ESP32 Snake Robot — Bluetooth SPP (8-Byte Packet Protocol)
  - RX: Exactly 8 bytes per command frame
  - TX: 12-byte telemetry frames (dir, x, y)
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "BluetoothSerial.h"
#include "Bitcraze_PMW3901.h"
#include <Adafruit_BNO08x.h>
#include <ESP32Servo.h>

// =====================================================
// Pins & Constants
// =====================================================
#define BNO08X_CS    15
#define BNO08X_INT   22
#define BNO08X_RESET  4
#define FLOW_CS       5

static const int kNumServos = 5;
static const int kServoPins[kNumServos] = {13, 12, 14, 27, 26};

enum ControlMode { MODE_M, MODE_P, MODE_D };
ControlMode currentMode = MODE_M;

static const char* kBtName = "ESP32_Snake";
BluetoothSerial SerialBT;

Bitcraze_PMW3901 flow(FLOW_CS);
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

static bool motionEnabled = false;
static float targetAmplitudeDeg = 30.0f;
static float currentAmplitudeDeg = 30.0f;
static float targetTurnOffsetDeg = 0.0f;
static float currentTurnOffsetDeg = 0.0f;

static float xw_counts = 0.0f;
static float yw_counts = 0.0f;
static bool  yawInitialized = false;
static float yaw0_deg = 0.0f;
static float lastYawDeg = 0.0f;
static float theta_deg = 0.0f;

static const float meters_per_count = 1.0f / 108.0f;
static unsigned long startTimeMs = 0;
static unsigned long lastLoopMs = 0;
static uint32_t lastTelemetryMs = 0;
static const uint32_t kTelemetryPeriodMs = 100;

Servo servos[kNumServos];

// =====================================================
// Helpers
// =====================================================
static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }
static inline float wrapDeg(float a) { while (a > 180.0f) a -= 360.0f; while (a < -180.0f) a += 360.0f; return a; }

static inline float slewToward(float current, float target, float maxStep) {
    float diff = target - current;
    if (diff > maxStep) diff = maxStep;
    if (diff < -maxStep) diff = -maxStep;
    return current + diff;
}

void calibrateToCenter() {
    for (int i = 0; i < kNumServos; i++) servos[i].write(90);
}

void writeFloatLE(BluetoothSerial& bt, float v) {
    uint8_t b[4];
    memcpy(b, &v, 4);
    bt.write(b, 4);
}

void processManualCommand(char c) {
    if (c == 'c' || c == 'C') {
        targetTurnOffsetDeg = 0.0f;
        currentTurnOffsetDeg = 0.0f;
        calibrateToCenter();
        motionEnabled = false;
    } else if (c == 'l' || c == 'L') {
        targetTurnOffsetDeg = 15.0f;
    } else if (c == 'r' || c == 'R') {
        targetTurnOffsetDeg = -15.0f;
    } else if (c == 's' || c == 'S') {
        targetTurnOffsetDeg = 0.0f;
        motionEnabled = true;
    } else if (c >= '0' && c <= '9') {
        targetAmplitudeDeg = clampf((c - '0') * 5.0f, 0.0f, 60.0f);
    } else if (c == '+') {
        targetAmplitudeDeg = clampf(targetAmplitudeDeg + 5.0f, 0.0f, 60.0f);
    } else if (c == '-') {
        targetAmplitudeDeg = clampf(targetAmplitudeDeg - 5.0f, 0.0f, 60.0f);
    }
}

void handleBluetoothRx() {
    while (SerialBT.available() >= 8) {
        uint8_t packet[8];
        SerialBT.readBytes(packet, 8);
        char modeChar = (char)packet[0];

        if (modeChar == 'M') {
            if (currentMode != MODE_M) { currentMode = MODE_M; Serial.println(">> MODE: MANUAL"); }
            processManualCommand((char)packet[1]);
        } 
        else if (modeChar == 'P') {
            if (currentMode != MODE_P) { currentMode = MODE_P; Serial.println(">> MODE: AUTO_POSITION"); }
        } 
        else if (modeChar == 'D') {
            if (currentMode != MODE_D) { currentMode = MODE_D; Serial.println(">> MODE: AUTO_DIRECTION"); }
        }
    }
}

void updateSensorsAndPosition() {
    int16_t dx = 0, dy = 0;
    flow.readMotionCount(&dx, &dy);

    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float qr = sensorValue.un.gameRotationVector.real;
            float qi = sensorValue.un.gameRotationVector.i;
            float qj = sensorValue.un.gameRotationVector.j;
            float qk = sensorValue.un.gameRotationVector.k;
            
            // Corrected atan2 for Quaternion to Yaw
            float yaw = atan2f(2.0f * (qi * qj + qk * qr), 1.0f - 2.0f * (qj * qj + qk * qk));
            lastYawDeg = yaw * (180.0f / PI);
            
            if (!yawInitialized) { yaw0_deg = lastYawDeg; yawInitialized = true; }
        }
    }

    if (!yawInitialized) return;
    theta_deg = wrapDeg(lastYawDeg - yaw0_deg);
    float rad = theta_deg * (PI / 180.0f);
    
    float dXw = cosf(rad) * (float)dx - sinf(rad) * (float)dy;
    float dYw = sinf(rad) * (float)dx + cosf(rad) * (float)dy;
    xw_counts += dXw;
    yw_counts += dYw;
}

void updateMotion(float dtSec, unsigned long nowMs) {
    currentTurnOffsetDeg = slewToward(currentTurnOffsetDeg, targetTurnOffsetDeg, 30.0f * dtSec);
    currentAmplitudeDeg  = slewToward(currentAmplitudeDeg,  targetAmplitudeDeg,  60.0f * dtSec);

    float tSec = (nowMs - startTimeMs) / 1000.0f;
    float omega = TWO_PI * 0.5f; 
    float phaseShift = TWO_PI / kNumServos;

    for (int i = 0; i < kNumServos; i++) {
        float angle = (90.0f + currentTurnOffsetDeg) + currentAmplitudeDeg * sinf(omega * tSec + i * phaseShift);
        servos[i].write((int)(clampf(angle, 0, 180) + 0.5f));
    }
}

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10); // Wait for Serial Monitor
    Serial.println("\n--- Snake Robot Setup ---");

    SPI.begin(18, 19, 23);
    pinMode(FLOW_CS, OUTPUT); pinMode(BNO08X_CS, OUTPUT);
    digitalWrite(FLOW_CS, HIGH); digitalWrite(BNO08X_CS, HIGH);

    // Flow Sensor Init
    if (!flow.begin()) {
        Serial.println("CRITICAL ERROR: PMW3901 (Flow) initialization FAILED!");
        while(1) delay(100); 
    } else {
        Serial.println("Check: PMW3901 Flow Sensor OK");
    }

    // BNO08x IMU Init
    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        Serial.println("CRITICAL ERROR: BNO08x (IMU) initialization FAILED!");
        while(1) delay(100);
    } else {
        Serial.println("Check: BNO08x IMU OK");
        bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
    }

    // Bluetooth
    if (!SerialBT.begin(kBtName)) {
        Serial.println("CRITICAL ERROR: Bluetooth initialization FAILED!");
    } else {
        Serial.printf("Check: Bluetooth OK. Name: %s\n", kBtName);
    }

    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    for (int i = 0; i < kNumServos; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(kServoPins[i], 500, 2400);
    }
    
    calibrateToCenter();
    Serial.println("System Ready. Waiting for Yaw initialization...");
    startTimeMs = millis(); lastLoopMs = startTimeMs;
}

void loop() {
    handleBluetoothRx();
    updateSensorsAndPosition();

    unsigned long nowMs = millis();
    float dtSec = (nowMs - lastLoopMs) / 1000.0f;
    lastLoopMs = nowMs;

    if (currentMode == MODE_M && motionEnabled) {
        updateMotion(dtSec, nowMs);
    }

    if (nowMs - lastTelemetryMs >= kTelemetryPeriodMs) {
        lastTelemetryMs = nowMs;
        if (SerialBT.hasClient() && yawInitialized) {
            writeFloatLE(SerialBT, theta_deg);
            writeFloatLE(SerialBT, xw_counts * meters_per_count);
            writeFloatLE(SerialBT, yw_counts * meters_per_count);
        }
    }
    delay(20);
}