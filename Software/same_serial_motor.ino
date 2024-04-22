#include <Stepper.h>

Stepper myStepper0(  2048, 9,11, 10, 12);
Stepper myStepper1(  2048, 5,7, 6, 8);
Stepper myStepper2(  2048, 1,3, 2, 4);
// IN1,IN3,IN2,IN4

int motor_speed=10;

void setup() {
  myStepper0.setSpeed(motor_speed);
  myStepper1.setSpeed(motor_speed);
  myStepper2.setSpeed(motor_speed);
  Serial.begin(9600);
  
}

void loop() {
    // read single angle from raspberry pi
  while (Serial.available() == 0) {
    
  }
  // Serial.println(data);
  String data = Serial.readStringUntil('\n');
  String angleData = data.substring(0, data.indexOf('$'));
  String motorData = data.substring(data.indexOf('$') + 1);
  
  float angle = angleData.toFloat();
  float motor = motorData.toFloat();

  Serial.print("Motor and Angle ");
  Serial.println(motor);
  Serial.println(angle);

  int steps = round((abs(angle) * 2048) / 360);
  Serial.print("Steps ");
  Serial.println(steps);

  int m=(int)motor;

  if (angle < 0) {
    switch (m) {
      case 0:
        myStepper0.step(-steps);
        break;
      case 1:
        myStepper1.step(-steps);
        break;
      case 2:
        myStepper2.step(-steps);
        break;
      default: 
        Serial.println("oopsie");
    }
  }
  else {
    switch (m) {
      case 0:
        myStepper0.step(steps);
        break;
      case 1:
        myStepper1.step(steps);
        break;
      case 2:
        myStepper2.step(steps);
        break;
      default: 
        Serial.println("oopsie");
    }
  }
}
