// #define L0_YAW_MOTOR_DIR_PIN 2
// #define L0_YAW_MOTOR_STEP_PIN 3

// TEST INPUT STRING: 5$0#10$1#15$2#20$3# 

#define L0_YAW_MOTOR_DIR_PIN 5
#define L0_YAW_MOTOR_STEP_PIN 10
#define L0_PITCH_MOTOR_DIR_PIN 7
#define L0_PITCH_MOTOR_STEP_PIN 6

#define L1_YAW_MOTOR_DIR_PIN 8
#define L1_YAW_MOTOR_STEP_PIN 9
#define L1_PITCH_MOTOR_DIR_PIN 12
#define L1_PITCH_MOTOR_STEP_PIN 11

// #define STEPS_PER_REV_STEPPER 3200
// Configuration on driver - 1 0 0 0 1 0 :DDDDDDD - Note - with 800 steps :))))))
#define STEPS_PER_REV_GEARBOX 32000

int step_pin = -1;
int dir_pin = -1;

String data = "";
String angle_data = "";
String motor_data = "";
String imu_data = "";

float angles[2] = {0.0, 0.0};
int steps[2] = {0,0};

int calculate_steps(int angle) {
  int div = 360 / abs(angle);
  int total_steps = STEPS_PER_REV_GEARBOX / div;

  return total_steps;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(L0_YAW_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L0_YAW_MOTOR_STEP_PIN, OUTPUT);
  pinMode(L0_PITCH_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L0_PITCH_MOTOR_STEP_PIN, OUTPUT);
  pinMode(L1_YAW_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L1_YAW_MOTOR_STEP_PIN, OUTPUT);
  pinMode(L1_PITCH_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L1_PITCH_MOTOR_STEP_PIN, OUTPUT);

  digitalWrite(L0_YAW_MOTOR_DIR_PIN, HIGH);
  digitalWrite(L0_PITCH_MOTOR_DIR_PIN, HIGH);
  digitalWrite(L1_YAW_MOTOR_DIR_PIN, HIGH);
  digitalWrite(L1_PITCH_MOTOR_DIR_PIN, HIGH);

  Serial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() == 0) {
    // Busy wait for angle 
  }

//  data = Serial.readStringUntil('\n');
//
//  Serial.print("Read data - ");
//  Serial.println(data);
//
//  angle_data = data.substring(0, data.indexOf('$'));
//  motor_data = data.substring(data.indexOf('$') + 1);
//
//  Serial.println(angle_data);
//  Serial.println(motor_data);
//
//  float angle = angle_data.toFloat();
//  float motor = motor_data.toFloat();

  //read command
  String command = Serial.readStringUntil('\n');
  // Format is Angle$MotorID#Angle$MotorID#Angle$MotorID#Angle$MotorID#

  Serial.print("Read command ");
  Serial.println(command);

  float angle = -1.0;

  for (int i = 0; i < 4; i++){
    if (command == '\n'){
      Serial.println("Invalid Command!!");
      return;
    }
    delayMicroseconds(1000);
//    Serial.println(i);
//    Serial.print("Command: ");
//    Serial.println(command);
//    Serial.println(command.length());
    unsigned int dollar_idx = command.indexOf('$');
    unsigned int hash_idx = command.indexOf('#');
//    Serial.print("Dollar index ");
//    Serial.print(dollar_idx);
//    Serial.print(", Hash index ");
//    Serial.println(hash_idx);

    //get angle
    String motor_angle = command.substring(0, dollar_idx);
//    Serial.print("Angle ");
//    Serial.println(motor_angle);

    //get motor ID
    String motor_id = command.substring(dollar_idx+1, hash_idx);
//    Serial.print("Motor ID ");
//    Serial.println(motor_id);

    //update angles array
    int id = motor_id.toInt();
    angle = motor_angle.toFloat();

    angles[id] = angle;

//     //update command
//    if (i < 3){
//      command = command.substring(hash_idx+1);
//    }
//    else{
//      command = command;
//    }
    command = command.substring(hash_idx+1);
//    Serial.print("New Command: ");
//    Serial.println(command);

//    Serial.println("----------------------");
    
  }

  Serial.print("Updated angles array: ");
  for (int j = 0; j < 4; j++){
    Serial.print(angles[j]);
    Serial.print(", ");
  }
  Serial.println("");

  for (int i = 0; i < 4; i++){
    if (i == 0) {
    step_pin = L0_YAW_MOTOR_STEP_PIN;
    dir_pin = L0_YAW_MOTOR_DIR_PIN;
    }
    else if (i == 1){
      step_pin = L0_PITCH_MOTOR_STEP_PIN;
      dir_pin = L0_PITCH_MOTOR_DIR_PIN;
    }
    else if (i == 2) {
      step_pin = L1_YAW_MOTOR_STEP_PIN;
      dir_pin = L1_YAW_MOTOR_DIR_PIN;
    }
    else {
      step_pin = L1_PITCH_MOTOR_STEP_PIN;
      dir_pin = L1_PITCH_MOTOR_DIR_PIN;
    }

    if (angles[i] < 0) {
      digitalWrite(dir_pin, LOW);
    } else {
      digitalWrite(dir_pin, HIGH);
    }

    int total_steps = calculate_steps(angle);

    for (int i = 0; i < total_steps; i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(500);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(500);
    }
    
  }

//  if (motor == 0) {
//    step_pin = L0_YAW_MOTOR_STEP_PIN;
//    dir_pin = L0_YAW_MOTOR_DIR_PIN;
//  }
//  else if (motor == 1){
//    step_pin = L0_PITCH_MOTOR_STEP_PIN;
//    dir_pin = L0_PITCH_MOTOR_DIR_PIN;
//  }
//  else if (motor == 2) {
//    step_pin = L1_YAW_MOTOR_STEP_PIN;
//    dir_pin = L1_YAW_MOTOR_DIR_PIN;
//  }
//  else {
//    step_pin = L1_PITCH_MOTOR_STEP_PIN;
//    dir_pin = L1_PITCH_MOTOR_DIR_PIN;
//  }
  
//  if (angle < 0) {
//    digitalWrite(dir_pin, LOW);
//  } else {
//    digitalWrite(dir_pin, HIGH);
//  }

//  int total_steps = calculate_steps(angle);
//
//  for (int i = 0; i < total_steps; i++) {
//    digitalWrite(step_pin, HIGH);
//    delayMicroseconds(500);
//    digitalWrite(step_pin, LOW);
//    delayMicroseconds(500);
//  }

  Serial.println("ok");
  delay(1000);
}
