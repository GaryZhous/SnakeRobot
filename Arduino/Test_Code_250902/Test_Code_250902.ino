#include <Servo.h>
#include <math.h>

// === CONFIGURATION ===
const int numServos = 1;
const int servoPins[numServos] = {3};

// === Motion parameters ===
float amplitude = 45;        // swing size (degrees)
float frequency = 0.5;       // Hz
float phaseShift = M_PI / 3; // radians
float center = 90;           // neutral position is 90°

unsigned long startTime;
int updateDelay = 20;

Servo servos[numServos];

void setup() {
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
  }
  startTime = millis();
}

void loop() {
  float t = (millis() - startTime) / 1000.0;

  for (int i = 0; i < numServos; i++) {
    // swing from (90 - A) to (90 + A)
    float angle = center + amplitude * sin(2 * M_PI * frequency * t + i * phaseShift);
    servos[i].write(angle);
  }

  delay(updateDelay);
}
