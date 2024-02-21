#include <Servo.h>

// Define servo motor parameters
Servo servoMotor1;
Servo servoMotor2;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Attach servo to pin 9
  servoMotor1.attach(9);
  servoMotor2.attach(10);

  servoMotor1.write(0);
  servoMotor2.write(0);
}

void loop() {
  // Read commands from Raspberry Pi
  Serial.print("OK");
  while (Serial.available() == 0);
  // if (Serial.available() > 0) {
    String command = Serial.readString();
    int pos = 0;
    
    // Actuate servo motor based on command
    if (command == "rotate_left") {
      for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servoMotor1.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        servoMotor1.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
    } else if (command == "rotate_right") {
        for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
          // in steps of 1 degree
          servoMotor2.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);                       // waits 15 ms for the servo to reach the position
        }
        for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
          servoMotor2.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);                       // waits 15 ms for the servo to reach the position
        }
    } 
  // }
}

