#define L0_YAW_MOTOR_DIR_PIN 2
#define L0_YAW_MOTOR_STEP_PIN 3
#define L0_PITCH_MOTOR_DIR_PIN 4
#define L0_PITCH_MOTOR_STEP_PIN 5

#define L1_YAW_MOTOR_DIR_PIN 7
#define L1_YAW_MOTOR_STEP_PIN 6
#define L1_PITCH_MOTOR_DIR_PIN 8
#define L1_PITCH_MOTOR_DSTEP_PIN 9

void setup() {
  // put your setup code here, to run once:
  pinMode(L0_YAW_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L0_YAW_MOTOR_STEP_PIN, OUTPUT);
  pinMode(L0_PITCH_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L0_PITCH_MOTOR_STEP_PIN, OUTPUT);
  pinMode(L1_YAW_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L1_YAW_MOTOR_STEP_PIN, OUTPUT);
  pinMode(L1_PITCH_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L1_PITCH_MOTOR_DSTEP_PIN, OUTPUT);

  digitalWrite(L0_YAW_MOTOR_DIR_PIN, HIGH);
  digitalWrite(L0_PITCH_MOTOR_DIR_PIN, HIGH);
  digitalWrite(L1_YAW_MOTOR_DIR_PIN, HIGH);
  digitalWrite(L1_PITCH_MOTOR_DIR_PIN, HIGH);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

}
