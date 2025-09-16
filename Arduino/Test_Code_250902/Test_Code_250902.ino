#include <Servo.h>
#include <math.h>

// === CONFIGURATION ===
const int numServos = 3;                     // Number of servo motors
const int servoPins[numServos] = {3, 5, 6}; // Update to your wiring

// === Motion parameters ===
float amplitude = 30;        // [degrees] Swing from center (recommended 15–40)
float frequency = 0.5;       // [Hz] Oscillation speed (recommended 0.3–1.0)
float phaseShift = M_PI / 3; // [radians] 60° phase offset between joints
float center = 90;           // [degrees] Neutral servo angle

// === Timing ===
unsigned long startTime;     // Used to track animation time
int updateDelay = 20;        // [ms] Loop delay (~50 Hz update rate)

// === Servo objects ===
Servo servos[numServos];

void setup() {
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
  }
  startTime = millis();
}

void loop() {
  // Convert elapsed time to seconds
  float t = (millis() - startTime) / 1000.0;

  // Drive each servo with sine wave
  for (int i = 0; i < numServos; i++) {
    float angle = center + amplitude * sin(2 * M_PI * frequency * t + i * phaseShift);
    servos[i].write(angle);
  }

  delay(updateDelay); // Control update rate
}
