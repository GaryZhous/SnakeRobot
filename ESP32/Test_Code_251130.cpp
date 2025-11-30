#include <ESP32Servo.h>
#include <math.h>

// ===== CONFIG =====
const int numServos = 6;

// Choose valid ESP32 GPIO pins for PWM output
// Example: change these to the pins you actually wired
const int servoPins[numServos] = {13, 12, 14, 27, 26, 25};

// traveling wave: phase spacing for 6 joints
const float phaseShift = TWO_PI / numServos; // = 2π/6 = 60°

// params
float amplitude = 45;        // swing size (degrees)
float frequency = 0.5;       // Hz
float center = 90;           // neutral position is 90°

unsigned long startTime;
int updateDelay = 20;        // ms

Servo servos[numServos];
int incomingByte = -1;       // for Serial commands

// Clamp and round angle to [0, 180]
int adjustAngle(float a) {
  if (a < 0)   a = 0;
  if (a > 180) a = 180;
  return (int)(a + 0.5f);
}

void calibrate() {
  for (int i = 0; i < numServos; i++) {
    servos[i].write(90); // align the motors properly first
  }
}

void setup() {
  Serial.begin(115200);

  // Allocate PWM timers (needed for ESP32Servo)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < numServos; i++) {
    servos[i].setPeriodHertz(50);           // standard servo frequency
    servos[i].attach(servoPins[i], 500, 2400); // pin, minUs, maxUs
  }

  startTime = millis();
  calibrate();
}

void loop() {
  float t = (millis() - startTime) / 1000.0f;
  float omega = TWO_PI * frequency;

  // Check serial command once per loop
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    // Expecting character '0' from Serial Monitor
    if (incomingByte == '0') {
      calibrate();
      // Optional: clear any extra '0's in the buffer
      while (Serial.available() > 0 && Serial.peek() == '0') {
        Serial.read();
      }
    }
  }

  for (int i = 0; i < numServos; i++) {
    float phase = i * phaseShift;
    float angle = center + amplitude * sinf(omega * t + phase);
    servos[i].write(adjustAngle(angle));
  }

  delay(updateDelay);
}
