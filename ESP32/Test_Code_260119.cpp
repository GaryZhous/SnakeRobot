#include <ESP32Servo.h>
#include <math.h>
#include "BluetoothSerial.h"

// configurations
const int numServos = 5;

// GPIO pins for the servos
const int servoPins[numServos] = {13, 12, 14, 27, 26};

// traveling wave: phase spacing for 5 joints
const float phaseShift = TWO_PI / numServos; // 2π/5

// motion params
float frequency = 0.5;       // Hz
float center = 90;           // neutral position

// Amplitude: target vs current (smoothed)
float targetAmplitude = 30;   // set by BT
float currentAmplitude = 30;  // actually applied (smooth)

// Turning: target vs current (smoothed)
float targetTurnOffset = 0;   // what you want (set by BT)
float currentTurnOffset = 0;  // what you actually apply (smooth)

// Turn tuning
float turnStepPerSec = 30.0f;   // deg/sec

// Amplitude tuning
float ampStepPerSec  = 60.0f;   // deg/sec (adjust to taste)
const float ampMin = 0.0f;
const float ampMax = 60.0f;

unsigned long startTime;
int updateDelay = 20;        // ms

Servo servos[numServos];
int incomingByte = -1;

BluetoothSerial SerialBT;

// motion state
bool motionEnabled = false;

// Smoothing time bookkeeping
unsigned long lastLoopMs = 0;

// Clamp and round angle to [0, 180]
int adjustAngle(float a) {
  if (a < 0)   a = 0;
  if (a > 180) a = 180;
  return (int)(a + 0.5f);
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void calibrate() {
  for (int i = 0; i < numServos; i++) {
    servos[i].write(90);
  }
}

// Slew-limit currentTurnOffset toward targetTurnOffset
void updateTurnSmoothing(float dtSec) {
  float maxStep = turnStepPerSec * dtSec;
  float diff = targetTurnOffset - currentTurnOffset;

  if (diff >  maxStep) diff =  maxStep;
  if (diff < -maxStep) diff = -maxStep;

  currentTurnOffset += diff;
}

// Slew-limit currentAmplitude toward targetAmplitude
void updateAmpSmoothing(float dtSec) {
  float maxStep = ampStepPerSec * dtSec;
  float diff = targetAmplitude - currentAmplitude;

  if (diff >  maxStep) diff =  maxStep;
  if (diff < -maxStep) diff = -maxStep;

  currentAmplitude += diff;
}

// Commands:
// 'c' -> calibrate + STOP (stay at 90 until 's')
// 'l' / 'L' -> turn left (smooth)
// 'r' / 'R' -> turn right (smooth)
// 's' / 'S' -> straight (smooth to 0) + RESUME motion
// '0'..'9' -> set amplitude = digit * 5 deg  (0..45)
// '+' / '-' -> amplitude +/- 5
void handleBluetooth() {
  while (SerialBT.available() > 0) {
    incomingByte = SerialBT.read();

    if (incomingByte == 'c' || incomingByte == 'C') {
      targetTurnOffset = 0;
      currentTurnOffset = 0;
      targetAmplitude = 0;
      currentAmplitude = 0;
      calibrate();
      motionEnabled = false;
      Serial.println("Calibrate to 90 deg, STOP");
      SerialBT.println("Calibrate to 90 deg, STOP");

    } else if (incomingByte == 'r' || incomingByte == 'R') {
      targetTurnOffset = -15;
      Serial.println("Turn LEFT (smooth)");
      SerialBT.println("Turn LEFT (smooth)");

    } else if (incomingByte == 'l' || incomingByte == 'L') {
      targetTurnOffset = 15;
      Serial.println("Turn RIGHT (smooth)");
      SerialBT.println("Turn RIGHT (smooth)");

    } else if (incomingByte == 's' || incomingByte == 'S') {
      targetTurnOffset = 0;
      motionEnabled = true;
      Serial.println("Straight & RESUME motion (smooth)");
      SerialBT.println("Straight & RESUME motion (smooth)");

    } else if (incomingByte >= '0' && incomingByte <= '9') {
      int d = incomingByte - '0';
      targetAmplitude = clampf(d * 5.0f, ampMin, ampMax);
      Serial.print("Set amplitude target to: ");
      Serial.println(targetAmplitude);
      SerialBT.print("Set amplitude target to: ");
      SerialBT.println(targetAmplitude);

    } else if (incomingByte == '+') {
      targetAmplitude = clampf(targetAmplitude + 5.0f, ampMin, ampMax);
      Serial.print("Amplitude target: ");
      Serial.println(targetAmplitude);
      SerialBT.print("Amplitude target: ");
      SerialBT.println(targetAmplitude);

    } else if (incomingByte == '-') {
      targetAmplitude = clampf(targetAmplitude - 5.0f, ampMin, ampMax);
      Serial.print("Amplitude target: ");
      Serial.println(targetAmplitude);
      SerialBT.print("Amplitude target: ");
      SerialBT.println(targetAmplitude);
    }
  }
}

void setup() {
  Serial.begin(115200);

  SerialBT.begin("ESP32_Snake");
  Serial.println("Bluetooth started, pair to 'ESP32_Snake'");

  pinMode(2, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(2, HIGH); delay(200);
    digitalWrite(2, LOW);  delay(200);
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < numServos; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 500, 2400);
  }

  startTime = millis();
  lastLoopMs = millis();
  calibrate();
}

void loop() {
  handleBluetooth();

  unsigned long now = millis();
  float dtSec = (now - lastLoopMs) / 1000.0f;
  lastLoopMs = now;

  if (motionEnabled) {
    // Smooth turning + amplitude every loop while moving
    updateTurnSmoothing(dtSec);
    updateAmpSmoothing(dtSec);

    float t = (now - startTime) / 1000.0f;
    float omega = TWO_PI * frequency;

    for (int i = 0; i < numServos; i++) {
      float phase = i * phaseShift;
      float angle = (center + currentTurnOffset) + currentAmplitude * sinf(omega * t + phase);
      servos[i].write(adjustAngle(angle));
    }
  }

  delay(updateDelay);
}
