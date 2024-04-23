// #define L0_YAW_MOTOR_DIR_PIN 2
// #define L0_YAW_MOTOR_STEP_PIN 3
#include <Arduino.h>

#define FRONT_LIMIT_SWITCH 2
#define BACK_LIMIT_SWITCH 3

#define L0_YAW_MOTOR_DIR_PIN 5
#define L0_YAW_MOTOR_STEP_PIN 10
#define L0_PITCH_MOTOR_DIR_PIN 7
#define L0_PITCH_MOTOR_STEP_PIN 6

#define L1_YAW_MOTOR_DIR_PIN 8
#define L1_YAW_MOTOR_STEP_PIN 9
#define L1_PITCH_MOTOR_DIR_PIN 12
#define L1_PITCH_MOTOR_STEP_PIN 11

#define STEP_DELAY 100  // Delay between steps in microseconds

#define MOTOR_PINS_SIZE 8
#define NUM_MOTORS 4
#define PINS_PER_MOTOR 2

// #define STEPS_PER_REV_STEPPER 3200
// Configuration on driver - 1 0 0 0 1 0 :DDDDDDD - Note - with 800 steps :))))))
#define STEPS_PER_REV_GEARBOX 32000

#define SWITCH_DEBOUNCE_TIME 50 // Debounce time in milliseconds

// Front switches map to pin3
// Back switches map to pin2

volatile bool hit_front = false;
volatile bool hit_back = false;

volatile bool new_data = false;

unsigned long last_interrupt_time_front = 0;
unsigned long last_interrupt_time_back = 0;

int step_pin = -1;
int dir_pin = -1;
int motor_id = -1;

String data = "";
String angle_data = "";
String motor_data = "";
String imu_data = "";

String prev_command = "";

int calculate_steps(int angle) {
  int div = 360 / abs(angle);
  int total_steps = STEPS_PER_REV_GEARBOX / div;

  return total_steps;
}

int motor_pins[NUM_MOTORS][PINS_PER_MOTOR] = { { L0_YAW_MOTOR_DIR_PIN, L0_YAW_MOTOR_STEP_PIN },
                    { L0_PITCH_MOTOR_DIR_PIN, L0_PITCH_MOTOR_STEP_PIN },
                    { L1_YAW_MOTOR_DIR_PIN, L1_YAW_MOTOR_STEP_PIN },
                    { L1_PITCH_MOTOR_DIR_PIN, L1_PITCH_MOTOR_STEP_PIN } };

long motor_steps[NUM_MOTORS] = {0, 0, 0, 0};

int stepCount = 0;      // Step counter for the leg

void handleFrontSwitch() {
  // This is called if either of the 2 front switches are hit
  // We don't really care about which one is hit, we check the array of motor_steps
  // and then just step the motor to the negative of those steps

  if (millis() - last_interrupt_time_front > SWITCH_DEBOUNCE_TIME) {
    hit_front = !hit_front; // Toggle the state
    last_interrupt_time_front = millis(); // Update the last interrupt time
  }
}

void handleBackSwitch() {
  if (millis() - last_interrupt_time_back > SWITCH_DEBOUNCE_TIME) {
    hit_back = !hit_back; // Toggle the state
    last_interrupt_time_back = millis(); // Update the last interrupt time
  }
}

void setup() {
  for (int i = 0; i < MOTOR_PINS_SIZE; i++) {
    for (int j = 0; j < PINS_PER_MOTOR; j++) {
      pinMode(motor_pins[i][j], OUTPUT);
    }
  }

  new_data=false;

  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_steps[i] = 0;
  }

  Serial.begin(9600);

  pinMode(FRONT_LIMIT_SWITCH, INPUT_PULLUP); // Use internal pull-up resistor
  pinMode(BACK_LIMIT_SWITCH, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(FRONT_LIMIT_SWITCH), handleFrontSwitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_LIMIT_SWITCH), handleBackSwitch, CHANGE);
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
  if (hit_front) {
    Serial.println("Front Limit Switch Triggered!");
    // Step back by 5 degrees 
    // 445 steps
    // If front motor is hit, it means one of the yaw motors went 
    // beyond the positive calibrated limit
    // Back off Buddy :)
    long motor0_steps = motor_steps[0];
    // TODO Check if greater than calibrated limit
    // reverse direction
    digitalWrite(motor_pins[0][0], LOW);
    step_pin = motor_pins[0][1];

    for (int i = 0; i < motor0_steps; i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(500);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(500);
    }

    digitalWrite(motor_pins[2][0], LOW);
    step_pin = motor_pins[2][1];

    long motor2_steps = motor_steps[2];

    for (int i = 0; i < motor0_steps; i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(500);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(500);
    }

    motor_steps[0] = 0;
    motor_steps[2] = 0;

    hit_front = false; // Reset the hit flag after handling
  }
  
  if (hit_back) {
    Serial.println("Back Limit Switch Triggered!");

    long motor0_steps = motor_steps[0];
    // TODO Check if greater than calibrated limit
    // reverse direction
    digitalWrite(motor_pins[0][0], HIGH);
    step_pin = motor_pins[0][1];

    for (int i = 0; i < motor0_steps; i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(500);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(500);
    }

    digitalWrite(motor_pins[2][0], HIGH);
    step_pin = motor_pins[2][1];

    long motor2_steps = motor_steps[2];

    for (int i = 0; i < motor0_steps; i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(500);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(500);
    }

    motor_steps[0] = 0;
    motor_steps[2] = 0;

    hit_back = false; // Reset the hit flag after handling
  }

  String data="";

  if (Serial.available() == 0) {
    return;
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

  }
  else if (motor == 1){
    step_pin = L0_PITCH_MOTOR_STEP_PIN;
    dir_pin = L0_PITCH_MOTOR_DIR_PIN;

    motor_id = (int)motor;
  }
  else if (motor == 2) {
    step_pin = L1_YAW_MOTOR_STEP_PIN;
    dir_pin = L1_YAW_MOTOR_DIR_PIN;

    motor_id = (int)motor;

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
