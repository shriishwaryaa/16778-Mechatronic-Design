#include <Servo.h> // Include the Servo library

Servo myServo;  // Create a Servo object

void setup() {
  myServo.attach(9); // Attach the servo signal pin to digital pin 9
  Serial.begin(9600);
}

void loop() {
  while (Serial.available() == 0) {

  }
  String data = Serial.readStringUntil('\n');
  Serial.println(data);

  float angle = data.toFloat();
  myServo.write(angle);
}
