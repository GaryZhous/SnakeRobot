#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>

Servo myServo;

void setup() {
  Serial.begin(115200);
  myServo.attach(9);  // Attach the servo on pin 9
}

void loop() {
  myServo.write(90);  // Move the servo to 90 degrees
  delay(1000);
  myServo.write(0);   // Move the servo to 0 degrees
  delay(1000);

}
