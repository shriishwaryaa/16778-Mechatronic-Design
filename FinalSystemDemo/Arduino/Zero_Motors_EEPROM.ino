#include <EEPROM.h>
#include <Arduino.h>
#include <assert.h>

#define L1_PITCH_MOTOR_DIR_PIN 12
#define L1_PITCH_MOTOR_STEP_PIN 11

#define LIMIT_SWITCH 2
#define SWITCH_DEBOUNCE_TIME 150 // Debounce time in milliseconds

volatile bool hit_yaw = false;
volatile bool hit_pitch = false;

volatile bool new_data = false;

unsigned long last_interrupt_time_yaw = 0;
unsigned long last_interrupt_time_pitch = 0;

// #define STEPS_PER_REV_STEPPER 3200
// Configuration on driver - 1 0 0 0 1 0 :DDDDDDD - Note - with 800 steps :))))))
#define STEPS_PER_REV_GEARBOX 32000

#define MOTOR_PINS_SIZE 2
#define NUM_MOTORS 1
#define PINS_PER_MOTOR 2

int step_pin = -1;
int dir_pin = -1;
int motor_id = -1;

String data = "";
String angle_data = "";
String motor_data = "";
String imu_data = "";

int eeprom_address = 0;

int motor_pins[NUM_MOTORS][PINS_PER_MOTOR] = { {L1_PITCH_MOTOR_DIR_PIN, L1_PITCH_MOTOR_STEP_PIN} };

long motor_steps[NUM_MOTORS] = {0};

int stepCount = 0;      // Step counter for the leg
int start_address = 0;

void write_positions_to_eeprom() {
  // Write the current motor positions to eeprom
  long current_steps=0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    current_steps = motor_steps[i];
    int addr = start_address + i * sizeof(long);
    Serial.println("Writing");
    Serial.println(current_steps);
    Serial.println(addr);
    EEPROM.put(addr, current_steps);
  }
}

int calculate_steps(int angle) {
  int div = 360 / abs(angle);
  int total_steps = STEPS_PER_REV_GEARBOX / div;

  return total_steps;
}

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < MOTOR_PINS_SIZE; i++) {
    for (int j = 0; j < PINS_PER_MOTOR; j++) {
      pinMode(motor_pins[i][j], OUTPUT);
    }
  }

  long steps=0;

  // Zero out all the motors
  Serial.println("Zeroing motors");

  for (int i = 0; i < NUM_MOTORS; i++) {
    long cur_steps;
    int addr = start_address + i * sizeof(long);
    Serial.println("EEPROM address");
    Serial.println(addr);
    EEPROM.get(addr, cur_steps);

    if (cur_steps==0) {
      continue;
    }

    Serial.print("Cur steps in setup ");
    Serial.println(cur_steps);

    if (cur_steps != 0) {
      if (cur_steps < 0) {
        digitalWrite(motor_pins[i][0], HIGH);
      } else {
        digitalWrite(motor_pins[i][0], LOW);
      }

      step_pin = motor_pins[i][1];

      cur_steps = abs(cur_steps);

      for (int i = 0; i < cur_steps; i++) {
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(500);
        digitalWrite(step_pin, LOW);
        delayMicroseconds(500);
      }
    }
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_steps[i] = 0;
  }
}

void loop() {
  String data="";

  if (Serial.available() == 0) {
    // write_positions_to_eeprom();
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
  motor_id=(int)motor;

  step_pin = L1_PITCH_MOTOR_STEP_PIN;
  dir_pin = L1_PITCH_MOTOR_DIR_PIN;
  
  if (angle < 0) {
    digitalWrite(dir_pin, LOW);
  }

  else {
    digitalWrite(dir_pin, HIGH);
  }

  int total_steps = calculate_steps(angle);
  Serial.print("Total steps");
  Serial.println(total_steps);

  if (angle >= 0) {
    motor_steps[motor_id] += total_steps;
    // Update EEPROM
  } else {
    motor_steps[motor_id] -= total_steps;
  }

  Serial.println("Updated motor steps ");
  Serial.print(motor_id);
  Serial.print("===>");
  Serial.println(motor_steps[motor_id]);

  int addr = start_address + motor_id * sizeof(long);
  EEPROM.put(addr, motor_steps[motor_id]);
  
  for (int i = 0; i < total_steps; i++) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(500);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(500);
  }

  Serial.println("ok");
  delay(1000);
  
}
