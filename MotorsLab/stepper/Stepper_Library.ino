#include <Stepper.h>

Stepper myStepper(  2048, 8,9, 10, 11);
// IN1,IN3,IN2,IN4

int motor_speed=10;

void setup() {
  myStepper.setSpeed(motor_speed);
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

  if (angle < 0) {
    myStepper.step(-steps);
  }
  else {
    myStepper.step(steps);
  }
}
