/*
  ESP32 Snake Robot — Bluetooth SPP (Strict 8-Byte Protocol)
  - RX: Exactly 8 bytes per packet.
  - TX: 12-byte telemetry frames (dir, x, y) float32 LE.
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
#define FLOW_RST      21

static const int kNumServos = 8;
static const int kServoPins[kNumServos] = {13, 12, 14, 27, 26, 25, 33, 32};
static const float meters_per_count = 1.0f / 108.0f;

// --- Control Modes & Gains ---
enum ControlMode { MODE_M, MODE_P, MODE_D };
ControlMode currentMode = MODE_M;

static int autoTargetYaw = 0; 
static const float Kp_yaw = 0.8f;        
static const float kMaxAutoTurn = 25.0f; 

static const char* kBtName = "ESP32_Snake";
BluetoothSerial SerialBT;

Bitcraze_PMW3901 flow(FLOW_CS);
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Motion State
static bool motionEnabled = false;
static float targetAmplitudeDeg = 30.0f;
static float currentAmplitudeDeg = 30.0f;
static float targetTurnOffsetDeg = 0.0f;
static float currentTurnOffsetDeg = 0.0f;

// Odometry
static float xw_counts = 0.0f;
static float yw_counts = 0.0f;
static bool  yawInitialized = false;
static float yaw0_deg = 0.0f;
static float lastYawDeg = 0.0f;
static float theta_deg = 0.0f;

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

// =====================================================
// Command Logic
// =====================================================
void processManualCommand(char c) {
    c = tolower(c);
    if (c == 'c') {
        targetTurnOffsetDeg = 0.0f;
        currentTurnOffsetDeg = 0.0f;
        calibrateToCenter();
        motionEnabled = false;
        Serial.println(">> Manual: CENTER & STOP");
    } else if (c == 'l') {
        targetTurnOffsetDeg = 15.0f;
        Serial.println(">> Manual: TURN LEFT");
    } else if (c == 'r') {
        targetTurnOffsetDeg = -15.0f;
        Serial.println(">> Manual: TURN RIGHT");
    } else if (c == 's') {
        targetTurnOffsetDeg = 0.0f;
        motionEnabled = true;
        Serial.println(">> Manual: START MOTION");
    } else if (c >= '0' && c <= '9') {
        targetAmplitudeDeg = clampf((c - '0') * 5.0f, 0.0f, 60.0f);
        Serial.printf(">> Manual: SET AMP TO %.1f\n", targetAmplitudeDeg);
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
        char input = (char)packet[0];

        // 1. Check for Mode Switch first
        if (input == 'M') {
            currentMode = MODE_M;
            Serial.println(">> SYSTEM: MODE -> MANUAL");
        } 
        else if (input == 'D') {
            currentMode = MODE_D;
            motionEnabled = true; 
            Serial.println(">> SYSTEM: MODE -> DIRECTION");
        }
        else if (input == 'P') {
            currentMode = MODE_P;
            Serial.println(">> SYSTEM: MODE -> POSITION");
        } 
        // 2. If not a Mode switch, and we are in Manual, process as Command
        else if (currentMode == MODE_M) {
            processManualCommand(input);
        }
        else if (currentMode == MODE_D) {

            if (packet[1] >= '0' && packet[1] <= '9' &&
                packet[2] >= '0' && packet[2] <= '9' &&
                packet[3] >= '0' && packet[3] <= '9') {
                int targetDirection = 0;

                targetDirection += (packet[1] - '0') * 100;
                targetDirection += (packet[2] - '0') * 10;
                targetDirection += (packet[3] - '0');

                autoTargetYaw = targetDirection;

                Serial.print("Target Angle:");
                Serial.println(autoTargetYaw);
            }

        }
    }
}

// =====================================================
// Sensors & Motion Update
// =====================================================
void updateSensorsAndPosition() {
    int16_t dx = 0, dy = 0;
    flow.readMotionCount(&dx, &dy);

    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float qr = sensorValue.un.gameRotationVector.real;
            float qi = sensorValue.un.gameRotationVector.i;
            float qj = sensorValue.un.gameRotationVector.j;
            float qk = sensorValue.un.gameRotationVector.k;
            // Yaw formula
            float yaw = atan2f(2.0f * (qi * qj + qk * qr), 1.0f - 2.0f * (qj * qj + qk * qk));
            lastYawDeg = yaw * (180.0f / PI);
            if (!yawInitialized) { yaw0_deg = lastYawDeg; yawInitialized = true; }
        }
    }

    if (!yawInitialized) return;
    theta_deg = wrapDeg(lastYawDeg - yaw0_deg);
    float rad = theta_deg * (PI / 180.0f);
    xw_counts += cosf(rad) * (float)dx - sinf(rad) * (float)dy;
    yw_counts += sinf(rad) * (float)dx + cosf(rad) * (float)dy;
}

void updateMotion(float dtSec, unsigned long nowMs) {
    currentTurnOffsetDeg = slewToward(currentTurnOffsetDeg, targetTurnOffsetDeg, 30.0f * dtSec);
    currentAmplitudeDeg  = slewToward(currentAmplitudeDeg,  targetAmplitudeDeg,  60.0f * dtSec);

    float tSec = (nowMs - startTimeMs) / 1000.0f;
    for (int i = 0; i < kNumServos; i++) {
        float angle = (90.0f + currentTurnOffsetDeg) + currentAmplitudeDeg * sinf(TWO_PI * 0.5f * tSec + i * (TWO_PI / kNumServos));
        servos[i].write((int)(clampf(angle, 0, 180) + 0.5f));
        // Serial.print("Servo:");
        // Serial.println(i);
        // Serial.print("Angle:");
        // Serial.println(angle);
    }
}

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    Serial.println("\n--- Snake Robot Initializing ---");

    SPI.begin(18, 19, 23);

    pinMode(FLOW_CS, OUTPUT);
    pinMode(FLOW_RST, OUTPUT);
    digitalWrite(FLOW_RST, LOW);
    delay(10);
    digitalWrite(FLOW_RST, HIGH);

    pinMode(BNO08X_CS, OUTPUT);
    digitalWrite(FLOW_CS, HIGH); digitalWrite(BNO08X_CS, HIGH);

    if (!flow.begin()) { Serial.println("Flow Init Failed!"); while(1); }
    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) { Serial.println("IMU Init Failed!"); while(1); }
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);

    SerialBT.begin(kBtName);
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    for (int i = 0; i < kNumServos; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(kServoPins[i], 500, 2400);
    }
    calibrateToCenter();
    startTimeMs = millis(); lastLoopMs = startTimeMs;
    Serial.println("System Ready.");
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
    else if (currentMode == MODE_D && motionEnabled) {
        float yawError = wrapDeg(autoTargetYaw - theta_deg);
        targetTurnOffsetDeg = clampf(yawError * Kp_yaw, -kMaxAutoTurn, kMaxAutoTurn);
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
    //delay(20);
}