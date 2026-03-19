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
static const float centimeters_per_count_y = 1.0f / 110.0f; // from 1/108
static const float centimeters_per_count_x = 1.0f / 120.0f; // from 1/108

// --- Control Modes & Gains ---
enum ControlMode { MODE_M, MODE_P, MODE_D };
ControlMode currentMode = MODE_M;

static int autoTargetYaw = 0; 
static const float Kp_yaw = 0.2f;        
static const float kMaxAutoTurn = 15.0f; 

static const char* kBtName = "ESP32_Snake";
BluetoothSerial SerialBT;

Bitcraze_PMW3901 flow(FLOW_CS);
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Motion State
static bool motionEnabled = false;
static float targetAmplitudeDeg = 30.0f;
static float currentAmplitudeDeg = 25.0f; // from 30
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

// Position Targets
static float targetX_cm = 0.0f; // Target X in meters
static float targetY_cm = 0.0f; // Target Y in meters

Servo servos[kNumServos];

//Global declarations for averaged yaw
const int max_samples = 20;
const float period = 1000/0.5f; //In ms
const float sampling_period = 2000/max_samples;
unsigned long prev_sample_time = 0;
int sample_idx = 0;
float yaw_samples [max_samples] = {0};
float x_samples [max_samples] = {0};
float y_samples [max_samples] = {0};
float sum_x;
float sum_y;
float avg_yaw = 0;

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
    //for (int i = 0; i < kNumServos; i++) servos[i].write(90);
    for (int i = (kNumServos/2)-1; i >= 0; i--) {
        servos[i].write(90);
        servos[(kNumServos-1)-i].write(90);
    }
}

void writeFloatLE(BluetoothSerial& bt, float v) {
    uint8_t b[4];
    memcpy(b, &v, 4);
    bt.write(b, 4);
}

float update_average(float yaw){ //Give it a clamped yaw IN RADIANS
    unsigned long cur_sample_time = millis();
    if (cur_sample_time-prev_sample_time>sampling_period){
        

        yaw_samples[sample_idx%max_samples] = yaw;
        sum_x=0;
        sum_y=0;
        for (int i=0; i<max_samples; i++){
            sum_x += cos(yaw_samples[i]);
            sum_y += sin(yaw_samples[i]);
        }

        avg_yaw = atan2(sum_y, sum_x);
        sample_idx++;
    }
    return avg_yaw;
}

void processManualCommand(char c, uint8_t packet[8]) {
    c = tolower(c);
    if (c == 'c') {
        targetTurnOffsetDeg = 0.0f;
        currentTurnOffsetDeg = 0.0f;
        calibrateToCenter();
        motionEnabled = false;
        Serial.println("Mode M: CENTER & STOP");
    } else if (c == '+') {
        targetAmplitudeDeg = clampf(targetAmplitudeDeg + 5.0f, 0.0f, 60.0f);
    } else if (c == '-') {
        targetAmplitudeDeg = clampf(targetAmplitudeDeg - 5.0f, 0.0f, 60.0f);
    } else if (c != 'D' && c != 'P' && c != 'M' && c != 'c'){

        motionEnabled = true;
        int manual_angle = 0;

        manual_angle += (packet[1] - '0') * 100;
        manual_angle += (packet[2] - '0') * 10;
        manual_angle += (packet[3] - '0');

        manual_angle -= 90;

        targetTurnOffsetDeg = (float) manual_angle;

        Serial.println("Mode M: angle received:");
        Serial.println(manual_angle);
        Serial.println();
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
            motionEnabled = false; 
            Serial.println(">> SYSTEM: MODE -> DIRECTION");
        }
        else if (input == 'P') {
            currentMode = MODE_P;
            motionEnabled = false;
            Serial.println(">> SYSTEM: MODE -> POSITION");
        } 
        // 2. If not a Mode switch, and we are in Manual, process as Command
        else if (currentMode == MODE_M) {
            processManualCommand(input, packet);
        }
        else if (currentMode == MODE_D) {


            if (packet[1] >= '0' && packet[1] <= '9' &&
                packet[2] >= '0' && packet[2] <= '9' &&
                packet[3] >= '0' && packet[3] <= '9') {


                motionEnabled = true;
                int targetDirection = 0;

                targetDirection += (packet[1] - '0') * 100;
                targetDirection += (packet[2] - '0') * 10;
                targetDirection += (packet[3] - '0');

                if (packet[0] == '-') {
                    targetDirection = -targetDirection;
                }

                autoTargetYaw = targetDirection;

                Serial.print("Mode D: Target Angle:");
                Serial.println(autoTargetYaw);
            }

        }
        else if (currentMode == MODE_P) {

            // 1. Parse X magnitude in meters (Bytes 1, 2, 3)
            if (packet[1] >= '0' && packet[1] <= '9' &&
                packet[2] >= '0' && packet[2] <= '9' &&
                packet[3] >= '0' && packet[3] <= '9' &&
                packet[5] >= '0' && packet[5] <= '9' &&
                packet[6] >= '0' && packet[6] <= '9' &&
                packet[7] >= '0' && packet[7] <= '9') {
                int parsedX_meters = 0;
                parsedX_meters += (packet[1] - '0') * 100;
                parsedX_meters += (packet[2] - '0') * 10;
                parsedX_meters += (packet[3] - '0');

                // Apply sign from Byte 0
                if (packet[0] == '-') {
                    parsedX_meters = -parsedX_meters;
                }

                // 2. Parse Y magnitude in meters (Bytes 5, 6, 7)
                int parsedY_meters = 0;
                parsedY_meters += (packet[5] - '0') * 100;
                parsedY_meters += (packet[6] - '0') * 10;
                parsedY_meters += (packet[7] - '0');

                // Apply sign from Byte 4
                if (packet[4] == '-') {
                    parsedY_meters = -parsedY_meters;
                }

                // 3. Convert meters to centimeters for internal tracking
                targetX_cm = (float)parsedX_meters;
                targetY_cm = (float)parsedY_meters;

                Serial.print("Target Pos (cm): X=");
                Serial.print(targetX_cm); 
                Serial.print(", Y="); 
                Serial.println(targetY_cm);
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

    dx = - dx;

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
    update_average(theta_deg*(PI/180.0f));
    // Serial.print("Average yaw: ");
    // Serial.print(avg_yaw*(180.0f/PI));
    // Serial.print(" Yaw: ");
    // Serial.println(theta_deg);

    avg_yaw = avg_yaw* (180.0f / PI);
    float rad = theta_deg * (PI / 180.0f);
    xw_counts += cosf(rad) * (float)dx - sinf(rad) * (float)dy;
    yw_counts += sinf(rad) * (float)dx + cosf(rad) * (float)dy;
}

void updateMotion(float dtSec, unsigned long nowMs) {
    if (!motionEnabled) return;
    currentTurnOffsetDeg = slewToward(currentTurnOffsetDeg, targetTurnOffsetDeg, 30.0f * dtSec);
    currentAmplitudeDeg  = slewToward(currentAmplitudeDeg,  targetAmplitudeDeg,  60.0f * dtSec);

    float tSec = (nowMs - startTimeMs) / 1000.0f;
    for (int i = 0; i < kNumServos; i++) {
        float angle = (90.0f - currentTurnOffsetDeg) + currentAmplitudeDeg * sinf(TWO_PI * 0.5f * tSec + i * (TWO_PI / kNumServos));
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
    //unsigned long loop_start_time = millis();
    handleBluetoothRx();
    updateSensorsAndPosition();

    unsigned long nowMs = millis();
    float dtSec = (nowMs - lastLoopMs) / 1000.0f;
    lastLoopMs = nowMs;

    if (currentMode == MODE_M) {
        updateMotion(dtSec, nowMs);
    } 
    else if (currentMode == MODE_D) {
        float yawError = wrapDeg(autoTargetYaw - theta_deg);
        targetTurnOffsetDeg = clampf(yawError * Kp_yaw, -kMaxAutoTurn, kMaxAutoTurn);
        updateMotion(dtSec, nowMs);
        Serial.print("Mode D: Target ");
        Serial.println(autoTargetYaw);
        Serial.print("Mode D: yaw error ");
        Serial.println(yawError);
        Serial.print("Mode D: current ");
        Serial.println(theta_deg);
        Serial.print("Mode D: turn offset ");
        Serial.println(targetTurnOffsetDeg);
    }
    else if (currentMode == MODE_P){

        // 1. Get current position in centimeters
        float currentX_cm = xw_counts * centimeters_per_count_x;
        float currentY_cm = yw_counts * centimeters_per_count_y;

        Serial.print("Mode P: Current x ");
        Serial.print(currentX_cm);
        Serial.print(" Current y ");
        Serial.print(currentY_cm);
        Serial.println();

        // 2. Calculate the difference (dx, dy)
        float dx = targetX_cm - currentX_cm;
        float dy = targetY_cm - currentY_cm;

        Serial.print("Mode P: Target x ");
        Serial.print(targetX_cm);
        Serial.print(" Target y ");
        Serial.print(targetY_cm);
        Serial.println();

        Serial.print("Mode P: Diff x ");
        Serial.print(dx);
        Serial.print(" Diff y ");
        Serial.print(dy);
        Serial.println();

        if ((dx < 20 && dx > -20) && (dy < 20 && dy > -20)){
            motionEnabled = false;
        } else {
            motionEnabled = true;
        }

        // 3. Calculate target angle using atan2 (returns radians, convert to degrees)
        float calculatedTargetYaw = atan2f(dy, dx) * (180.0f / PI) - 90;

        Serial.print("Angle ");
        Serial.print(calculatedTargetYaw);

        // 4. Calculate error and steer
        float yawError = wrapDeg(calculatedTargetYaw - theta_deg);
        targetTurnOffsetDeg = clampf(yawError * Kp_yaw, -kMaxAutoTurn, kMaxAutoTurn);
        
        updateMotion(dtSec, nowMs);
            
    }

    if (nowMs - lastTelemetryMs >= kTelemetryPeriodMs) {
        lastTelemetryMs = nowMs;
        if (SerialBT.hasClient() && yawInitialized) {
            // writeFloatLE(SerialBT, theta_deg);
            writeFloatLE(SerialBT, avg_yaw);
            writeFloatLE(SerialBT, xw_counts * centimeters_per_count_x);
            writeFloatLE(SerialBT, yw_counts * centimeters_per_count_y);
        }
    }
    delay(20);
    //unsigned long loop_end_time = millis();
    // Serial.print("Loop time:");
    // Serial.println(loop_end_time-loop_start_time);
}
