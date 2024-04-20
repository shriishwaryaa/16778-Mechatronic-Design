/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-limit-switch
 */

#include <ezButton.h>

#define L0_YAW_MOTOR_DIR_PIN 2
#define L0_YAW_MOTOR_STEP_PIN 3

#define STEP_DELAY 100  // Delay between steps in microseconds

ezButton limitSwitchLeft(7);  // create ezButton object that attach to pin 7;
ezButton limitSwitchRight(8);

bool hitLeft = false;
bool hitRight = false;

bool stateLeft = false;
bool stateRight = false;

#define STEPS_PER_REV_GEARBOX 32000

int step_pin = -1;
int dir_pin = -1;
int motor_id = -1;

int calculate_steps(int angle) {
  int div = 360 / abs(angle);
  int total_steps = STEPS_PER_REV_GEARBOX / div;

  return total_steps;
}


void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  limitSwitch1.setDebounceTime(50); // set debounce time to 50 milliseconds
  limitSwitch2.setDebounceTime(50); // set debounce time to 50 milliseconds
}

void loop() {
  limitSwitchLeft.loop(); // MUST call the loop() function first
  limitSwitchRight.loop(); // MUST call the loop() function first

// Toggle button state when shift in state
  if(limitSwitchLeft.isReleased())
    Serial.println("The limit switch1 : UNTOUCHED -> TOUCHED");
    hitLeft = true;

  if(limitSwitchLeft.isPressed())
    Serial.println("The limit switch1 : TOUCHED -> UNTOUCHED");

  if(limitSwitchRight.isReleased())
    Serial.println("The limit switch 2: UNTOUCHED -> TOUCHED");
    hitRight = true;

  if(limitSwitchRight.isPressed())
    Serial.println("The limit switch 2: TOUCHED -> UNTOUCHED");

  // Get current button state
  stateLeft = limitSwitchLeft.getState();
  stateRight = limitSwitchRight.getState();
  
  // toggle currButton state back
  if(stateLeft == LOW)
    Serial.println("The limit switch1: UNTOUCHED");
    hitLeft = false;

  if(stateRight == LOW)
    Serial.println("The limit switch1: UNTOUCHED");
    hitRight = false;

}
