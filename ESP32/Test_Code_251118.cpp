#include <WiFi.h>
#include <Servo.h>
#include <math.h>

// config
const char* ssid = "yourNetworkName";
const int numServos = 3; // 3 for testing for now
const int servoPins[numServos] = {3, 5, 6}; // pwm pins are 3, 5, 6, 9, 10, 11
// traveling wave: 120° phase spacing for 3 joints
const float phaseShift = TWO_PI / numServos; // = 2π/3 for 3 servos

// params
float amplitude = 45;        // swing size (degrees)
float frequency = 0.5;       // Hz
float phaseShift = M_PI / 3; // radians
float center = 90;           // neutral position is 90°

unsigned long startTime;
int updateDelay = 20;

Servo servos[numServos];
void setup(){
Serial.begin(115200);
delay(1000);
WiFi.begin(ssid);
Serial.println("\nConnecting");

while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
}

Serial.println("\nConnected to the WiFi network");
Serial.print("Local ESP32 IP: ");
Serial.println(WiFi.localIP());
for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
  }
  startTime = millis();
}

int adjustAngle(float a) {
  if (a < 0)   
    a = 0;
  if (a > 180) 
    a = 180;
  return (int)(a + 0.5f); // make it an integer
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
