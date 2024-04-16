#define L0_YAW_MOTOR_DIR_PIN 2
#define L0_YAW_MOTOR_STEP_PIN 3
#define L0_PITCH_MOTOR_DIR_PIN 7
#define L0_PITCH_MOTOR_STEP_PIN 6

#define L1_YAW_MOTOR_DIR_PIN 8
#define L1_YAW_MOTOR_STEP_PIN 9
#define L1_PITCH_MOTOR_DIR_PIN 12
#define L1_PITCH_MOTOR_STEP_PIN 11

#define STEP_DELAY 100  // Delay between steps in microseconds

#define MOTOR_PINS_SIZE 8
#define NUM_MOTORS 4

// #define STEPS_PER_REV_STEPPER 3200
// Configuration on driver - 1 0 0 0 1 0 :DDDDDDD - Note - with 800 steps :))))))
#define STEPS_PER_REV_GEARBOX 32000

int step_pin = -1;
int dir_pin = -1;
int motor_id = -1;

String data = "";
String angle_data = "";
String motor_data = "";
String imu_data = "";

int calculate_steps(int angle) {
  int div = 360 / abs(angle);
  int total_steps = STEPS_PER_REV_GEARBOX / div;

  return total_steps;
}

int motor_pins[MOTOR_PINS_SIZE] = {L0_YAW_MOTOR_DIR_PIN, L0_YAW_MOTOR_STEP_PIN,
                    L0_PITCH_MOTOR_DIR_PIN, L0_PITCH_MOTOR_STEP_PIN,
                    L1_YAW_MOTOR_DIR_PIN, L1_YAW_MOTOR_STEP_PIN,
                    L1_PITCH_MOTOR_DIR_PIN, L1_PITCH_MOTOR_STEP_PIN};

int motor_ids[NUM_MOTORS] = {0, 1, 2, 3};

long motor_steps[NUM_MOTORS] = {0, 0, 0, 0};

int stepCount = 0;      // Step counter for the leg

void setup() {
  for (int i = 0; i < MOTOR_PINS_SIZE; i++) {
    pinMode(motor_pins[i], OUTPUT);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_steps[i] = 0;
  }

  Serial.begin(9600);
}

void step_motor(int step_pin) {
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(STEP_DELAY);
}

void moveForward(int steps) {
  digitalWrite(dir_pin, LOW); // Set direction to forward
  for(int i = 0; i < steps; i++) {
    step_motor(step_pin);
    motor_steps[motor_id]++;  // Increment step counter
  }
}

void moveBackward(int steps) {
  digitalWrite(dir_pin, HIGH); // Set direction to backward
  for(int i = 0; i < steps; i++) {
    step_motor(step_pin);
    motor_steps[motor_id]--;  // Decrement step counter
  }
}

// Function to re-zero the robot
void reZero() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motor_steps[i] > 0) {
      moveBackward(motor_steps[i]);
    } else {
      moveForward(-motor_steps[i]);
    }
    motor_steps[i] = 0;
  }
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

  Serial.println(angle_data);
  Serial.println(motor_data);

  float angle = angle_data.toFloat();
  float motor = motor_data.toFloat();

  if (motor == 0) {
    step_pin = L0_YAW_MOTOR_STEP_PIN;
    dir_pin = L0_YAW_MOTOR_DIR_PIN;
    motor_id = (int)motor;

    // Serial.print("Motor ");
    // Serial.println(motor);
    
    // Serial.print("step pin: ");
    // Serial.println(step_pin);

    // Serial.print("dir pin: ");
    // Serial.println(dir_pin);
  }
  else if (motor == 1){
    step_pin = L0_PITCH_MOTOR_STEP_PIN;
    dir_pin = L0_PITCH_MOTOR_DIR_PIN;

    motor_id = (int)motor;
    // Serial.print("Motor ");
    // Serial.println(motor);

    // Serial.print("step pin: ");
    // Serial.println(step_pin);

    // Serial.print("dir pin: ");
    // Serial.println(dir_pin);
  }
  else if (motor == 2) {
    step_pin = L1_YAW_MOTOR_STEP_PIN;
    dir_pin = L1_YAW_MOTOR_DIR_PIN;

    motor_id = (int)motor;

    // Serial.print("Motor ");
    // Serial.println(motor);

    // Serial.print("step pin: ");
    // Serial.println(step_pin);

    // Serial.print("dir pin: ");
    // Serial.println(dir_pin);
  }
  else if (motor == 3) {
    step_pin = L1_PITCH_MOTOR_STEP_PIN;
    dir_pin = L1_PITCH_MOTOR_DIR_PIN;

    motor_id = (int)motor;

    // Serial.print("Motor ");
    // Serial.println(motor);

    // Serial.print("step pin: ");
    // Serial.println(step_pin);

    // Serial.print("dir pin: ");
    // Serial.println(dir_pin);
  }
  
  if (angle < 0) {
    digitalWrite(dir_pin, LOW);
  }

  else {
    digitalWrite(dir_pin, HIGH);
  }

  int total_steps = calculate_steps(angle);
  if (angle >= 0) {
    motor_steps[motor_id] += total_steps;
  } else {
    motor_steps[motor_id] -= total_steps;
  }
  
  for (int i = 0; i < total_steps; i++) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(500);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(500);
  }

  Serial.println("ok");
  delay(1000);
}
