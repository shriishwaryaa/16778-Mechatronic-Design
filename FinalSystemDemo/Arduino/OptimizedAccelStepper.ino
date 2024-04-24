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

int start_address = 0;
long motor_steps[NUM_MOTORS] = {0, 0, 0, 0};

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

int motor_pins[][2] = { 
    { L0_YAW_MOTOR_DIR_PIN, L0_YAW_MOTOR_STEP_PIN },
    { L0_PITCH_MOTOR_DIR_PIN, L0_PITCH_MOTOR_STEP_PIN },
    { L1_YAW_MOTOR_DIR_PIN, L1_YAW_MOTOR_STEP_PIN },
    { L1_PITCH_MOTOR_DIR_PIN, L1_PITCH_MOTOR_STEP_PIN }
};

AccelStepper steppers[4] = {
    AccelStepper(AccelStepper::DRIVER, L0_YAW_MOTOR_STEP_PIN, L0_YAW_MOTOR_DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, L0_PITCH_MOTOR_STEP_PIN, L0_PITCH_MOTOR_DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, L1_YAW_MOTOR_STEP_PIN, L1_YAW_MOTOR_DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, L1_PITCH_MOTOR_STEP_PIN, L1_PITCH_MOTOR_DIR_PIN)
};

void setup() {
    Serial.begin(9600);
    for (int i = 0; i < 4; i++) {
        steppers[i].setMaxSpeed(100);  // Set the maximum steps per second
        steppers[i].setAcceleration(500);  // Set the acceleration in steps per second squared
        long position;
        EEPROM.get(i * sizeof(long), position);
        
        if (position != 0) {
          steppers[i].moveTo(0);
          steppers[i].run();
        }
    }
}

void loop() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');
        int dollarIndex = data.indexOf('$');
        float angle = data.substring(0, dollarIndex).toFloat();
        int motor_id = data.substring(dollarIndex + 1).toInt();

        long steps = calculate_steps(angle);  // Example conversion, adjust as needed
        steppers[motor_id].moveTo(steps);
        steppers[motor_id].run();
        Serial.println("ok");

    } else {
      write_positions_to_eeprom();
      return;
    }

}