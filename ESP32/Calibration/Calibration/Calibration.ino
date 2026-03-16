#include <ESP32Servo.h>
#include <math.h>

// config
const int numServos = 8;
const int servoPins[numServos] =  {13, 12, 14, 27, 26, 25, 33, 32};

// params
float amplitude = 45.0;                  // swing size in degrees
float frequency = 0.5;                   // Hz
float phaseShift = TWO_PI / numServos;   // radians between servos
float center = 90.0;                     // neutral angle

unsigned long startTime;
const int updateDelay = 20;              // ms

Servo servos[numServos];

int adjustAngle(float a) {
  if (a < 0) a = 0;
  if (a > 180) a = 180;
  return (int)(a + 0.5f);
}

void setup() {
  // Optional, but helps with some ESP32 boards/libraries
  // ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  // ESP32PWM::allocateTimer(2);
  // ESP32PWM::allocateTimer(3);

  for (int i = 0; i < numServos; i++) {
    servos[i].setPeriodHertz(50);                    // standard servo frequency
    servos[i].attach(servoPins[i], 500, 2400);       // min/max pulse width in us
    servos[i].write(adjustAngle(center));            // start at center
  }

  startTime = millis();

}

void loop() {
}