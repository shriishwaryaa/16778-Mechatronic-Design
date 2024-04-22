// One motor on the Arduino controls 4 motors 


#include <Stepper.h>

Stepper myStepper0(2048, 9,11, 10, 12);
Stepper myStepper1(2048, 5,7, 6, 8);

Stepper stepper_array[2] = { myStepper0, myStepper1 };

// IN1,IN3,IN2,IN4

int motor_speed=10;

void setup() {
  stepper_array[0].setSpeed(motor_speed);
  stepper_array[1].setSpeed(motor_speed);
  // myStepper2.setSpeed(motor_speed);
  Serial.begin(9600);
  
}

float angles[2] = {0.0, 0.0};
int steps[2] = {0,0};

void loop() {
    // read single angle from raspberry pi
  while (Serial.available() == 0) {
    
  }
  // Serial.println(data);
  String command = Serial.readStringUntil('\n');
  // Format is 0$5$1$10

  Serial.print("Read command ");
  Serial.println(command);

  unsigned int dollar_idx = command.indexOf('$');
  Serial.print("Dollar index ");
  Serial.println(dollar_idx);
  // fetch motor ID
  String motor_id = command.substring(0, dollar_idx);
  Serial.print("Motor ID ");
  Serial.println(motor_id);
  // Update command
  command=command.substring(dollar_idx+1);
  // new delimiter index

  Serial.print("New command ");
  Serial.println(command);

  dollar_idx = command.indexOf('$');
  Serial.print("Dollar index ");
  Serial.println(dollar_idx);

  String motor_angle = command.substring(0,dollar_idx);
  Serial.print("Motor angle ");
  Serial.println(motor_angle);
  // Update command 
  command=command.substring(dollar_idx+1);
  Serial.print("New command ");
  Serial.println(command);

  int id = motor_id.toInt();
  float angle = motor_angle.toFloat();

  angles[id] = angle;
  dollar_idx = command.indexOf('$');
  // fetch motor ID
  motor_id = command.substring(0, dollar_idx);
  // Update command
  command=command.substring(dollar_idx+1);
  // new delimiter index
  dollar_idx = command.indexOf('$');
  motor_angle = command.substring(0,dollar_idx);
  // Update command 
  command=command.substring(dollar_idx+1);

  id = motor_id.toInt();
  angle = motor_angle.toFloat();
  angles[id] = angle;

  for (int i = 0; i < 2; i++) {
    int s = round((abs(angle) * 2048) / 360);
    steps[i] = s;
    Serial.print("Steps ");
    Serial.print(i);
    Serial.println(steps[i]);
  }

  for (int i = 0; i < 2; i++) {
    if (angles[i] < 0) {
      stepper_array[i].step(-steps[i]);
    }
    else {
      stepper_array[i].step(steps[i]);
    }
  }
}
