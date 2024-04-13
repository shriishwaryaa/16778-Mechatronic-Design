#define L0_YAW_MOTOR_DIR_PIN 2
#define L0_YAW_MOTOR_STEP_PIN 3
#define L0_PITCH_MOTOR_DIR_PIN 4
#define L0_PITCH_MOTOR_STEP_PIN 5

#define L1_YAW_MOTOR_DIR_PIN 7
#define L1_YAW_MOTOR_STEP_PIN 6
#define L1_PITCH_MOTOR_DIR_PIN 8
#define L1_PITCH_MOTOR_STEP_PIN 9

// #define STEPS_PER_REV_STEPPER 3200
// Configuration on driver - 1 0 0 0 1 0 :DDDDDDD - Note - with 800 steps :))))))
#define STEPS_PER_REV_GEARBOX 32000

int step_pin = -1;
int dir_pin = -1;

String data = "";
String angle_data = "";
String motor_data = "";
String imu_data = "";

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

  data = Serial.readStringUntil('\n');

  Serial.print("Read data - ");
  Serial.println(data);

  angle_data = data.substring(0, data.indexOf('$'));
  motor_data = data.substring(data.indexOf('$') + 1);

  float angle = angle_data.toFloat();
  float motor = motor_data.toFloat();

  if (motor == 0) {
    step_pin = L0_YAW_MOTOR_STEP_PIN;
    dir_pin = L0_YAW_MOTOR_DIR_PIN;
  }
  else if (motor == 1){
    step_pin = L0_PITCH_MOTOR_STEP_PIN;
    dir_pin = L0_PITCH_MOTOR_DIR_PIN;
  }
  else if (motor == 2) {
    step_pin = L1_YAW_MOTOR_STEP_PIN;
    dir_pin = L1_YAW_MOTOR_DIR_PIN;
  }
  else {
    step_pin = L1_PITCH_MOTOR_STEP_PIN;
    dir_pin = L1_PITCH_MOTOR_DIR_PIN;
  }
  
  if (angle < 0) {
    digitalWrite(dir_pin, LOW);
  }

  else {
    digitalWrite(dir_pin, HIGH);
  }

  int total_steps = calculate_steps(angle);

  for (int i = 0; i < total_steps; i++) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(500);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(500);
  }

  Serial.println("ok");
  delay(1000);
}
