#include <EEPROM.h>
#include <Arduino.h>
#include <AccelStepper.h>

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

#define MOTOR_PINS_SIZE 8
#define NUM_MOTORS 4
#define PINS_PER_MOTOR 2

int step_pin = -1;
int dir_pin = -1;
int motor_id = -1;

String data = "";
String angle_data = "";
String motor_data = "";
String imu_data = "";

int eeprom_address = 0;

int motor_pins[NUM_MOTORS][PINS_PER_MOTOR] = { { L0_YAW_MOTOR_DIR_PIN, L0_YAW_MOTOR_STEP_PIN },
                    { L0_PITCH_MOTOR_DIR_PIN, L0_PITCH_MOTOR_STEP_PIN },
                    { L1_YAW_MOTOR_DIR_PIN, L1_YAW_MOTOR_STEP_PIN },
                    { L1_PITCH_MOTOR_DIR_PIN, L1_PITCH_MOTOR_STEP_PIN } };

long motor_steps[NUM_MOTORS] = {0, 0, 0, 0};

int stepCount = 0;      // Step counter for the leg
int start_address = 0;

void write_positions_to_eeprom() {
  // Write the current motor positions to eeprom
  long current_steps=0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    current_steps = motor_steps[i];
    int addr = start_address + i * sizeof(long);
    EEPROM.put(addr, current_steps);
  }
}

int calculate_steps(int angle) {
  int div = 360 / abs(angle);
  int total_steps = STEPS_PER_REV_GEARBOX / div;

  return total_steps;
}

AccelStepper steppers[NUM_MOTORS];

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < MOTOR_PINS_SIZE; i++) {
    AccelStepper stepper(AccelStepper::DRIVER, motor_pins[i][1], motor_pins[i][0]);
    steppers[i] = stepper;

    // Change to go faster if Robot doesn't walk... :/
    stepper.setMaxSpeed(100);  // Set the maximum steps per second
    stepper.setAcceleration(500);  // Set the acceleration in steps per second squared
  }

  long steps=0;

  for (int i = 0; i < NUM_MOTORS; i++) {
    int addr = start_address + i * sizeof(long);
    EEPROM.get(addr, motor_steps[i]);
    steppers[i].moveTo(motor_steps[i]);
  }
}

void loop() {
  String data="";

  if (Serial.available() == 0) {
    write_positions_to_eeprom();
    return;
  }

  data = Serial.readStringUntil('\n');

  // Serial.print("Read data - ");
  // Serial.println(data);

  angle_data = data.substring(0, data.indexOf('$'));
  motor_data = data.substring(data.indexOf('$') + 1);

  // Serial.println(angle_data);
  // Serial.println(motor_data);

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

  int total_steps = calculate_steps(angle);
  steppers[motor_id].moveTo(total_steps);

  Serial.println("ok");
  delay(1000);
  
}
