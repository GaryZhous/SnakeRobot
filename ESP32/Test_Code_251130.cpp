#include <ESP32Servo.h>
#include <math.h>
#include "BluetoothSerial.h"   // have to import bruh

// ===== CONFIG =====
const int numServos = 6;

// Pick valid ESP32 GPIO pins
const int servoPins[numServos] = {13, 12, 14, 27, 26, 25}; // servo pins will need to adjust

// traveling wave: phase spacing for 6 joints
const float phaseShift = TWO_PI / numServos; // 2π/6 = 60°

// motion params
float amplitude = 30;        // swing size (degrees) reduced
float frequency = 0.5;       // Hz
float center = 90;           // neutral position
float turnOffset = 0;        // extra bias for turning (±deg)

unsigned long startTime;
int updateDelay = 20;        // ms

Servo servos[numServos];
int incomingByte = -1;       // for commands

BluetoothSerial SerialBT; 

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

// '0' -> calibrate
// 'l' / 'L' -> turn left
// 'r' / 'R' -> turn right
// 's' / 'S' -> straight (no turn)
void handleBluetooth() {
  while (SerialBT.available() > 0) {    
    incomingByte = SerialBT.read();

    if (incomingByte == '0') {
      turnOffset = 0;
      calibrate();
      Serial.println("Calibrate to 90 deg");
      SerialBT.println("Calibrate to 90 deg");
    } else if (incomingByte == 'l' || incomingByte == 'L') {
      turnOffset = -15;   // try negative for one side
      Serial.println("Turn LEFT");
      SerialBT.println("Turn LEFT");
    } else if (incomingByte == 'r' || incomingByte == 'R') {
      turnOffset = 15;    // positive for the other side
      Serial.println("Turn RIGHT");
      SerialBT.println("Turn RIGHT");
    } else if (incomingByte == 's' || incomingByte == 'S') {
      turnOffset = 0;
      Serial.println("Straight");
      SerialBT.println("Straight");
    }
  }
}

void setup() {
  Serial.begin(115200);                 

  SerialBT.begin("ESP32_Snake"); 
  Serial.println("Bluetooth started, pair to 'ESP32_Snake'"); // debugging

  // PWM timers for ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < numServos; i++) {
    servos[i].setPeriodHertz(50);              // standard servo frequency
    servos[i].attach(servoPins[i], 500, 2400);
  }

  startTime = millis();
  calibrate();
}

void loop() {
  handleBluetooth();   // check for new BT commands

  float t = (millis() - startTime) / 1000.0f;
  float omega = TWO_PI * frequency;

  for (int i = 0; i < numServos; i++) {
    float phase = i * phaseShift;
    // add turnOffset to bias the whole wave to one side
    float angle = (center + turnOffset) + amplitude * sinf(omega * t + phase);
    servos[i].write(adjustAngle(angle));
  }

  delay(updateDelay);
}
