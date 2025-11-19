#include <Servo.h>
#include <math.h>

// config
const int numServos = 6;
const int servoPins[numServos] = {11, 10, 9, 6, 5, 3};  // 6 PWM pins for 6 motors

// traveling wave: phase spacing for 6 joints
const float phaseShift = TWO_PI / numServos; // = 2π/6 = 60°

// params
float amplitude = 45;        // swing size (degrees)
float frequency = 0.5;       // Hz
float center = 90;           // neutral position is 90°

unsigned long startTime;
int updateDelay = 20;

Servo servos[numServos];

int adjustAngle(float a) {
  if (a < 0)   a = 0;
  if (a > 180) a = 180;
  return (int)(a + 0.5f); // round to nearest int
}

void setup() {
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
  }
  startTime = millis();
  for (int i = 0; i < numServos; i++){
    servos[i].write(90); // align the motors properly first
  }
}

void loop() {
  float t = (millis() - startTime) / 1000.0;
  float omega = TWO_PI * frequency;
  
  for (int i = 0; i < numServos; i++) {
    float phase = i * phaseShift;
    float angle = center + amplitude * sinf(omega * t + phase);
    servos[i].write(adjustAngle(angle));
  }

  delay(updateDelay);
}
