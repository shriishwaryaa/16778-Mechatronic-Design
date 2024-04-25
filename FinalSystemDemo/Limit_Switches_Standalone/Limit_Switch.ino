#include <Arduino.h>
#define SWITCH1 2
#define SWITCH2 3
#define SWITCH_DEBOUNCE_TIME 50

bool hit_front = true;
bool hit_back = true;
unsigned long last_interrupt_time_front = 0;
unsigned long last_interrupt_time_back = 0;

void handleFrontSwitch() {
  // This is called if either of the 2 front switches are hit
  // We don't really care about which one is hit, we check the array of motor_steps
  // and then just step the motor to the negative of those steps

  if (millis() - last_interrupt_time_back > SWITCH_DEBOUNCE_TIME) {
    hit_back = !hit_back; // Toggle the state
    last_interrupt_time_back = millis(); // Update the last interrupt time
  }
}

void handleBackSwitch() {
  // This is called if either of the 2 front switches are hit
  // We don't really care about which one is hit, we check the array of motor_steps
  // and then just step the motor to the negative of those steps

  if (millis() - last_interrupt_time_front > SWITCH_DEBOUNCE_TIME) {
    hit_front = !hit_front; // Toggle the state
    last_interrupt_time_front = millis(); // Update the last interrupt time
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SWITCH1, INPUT_PULLUP); // Use internal pull-up resistor
  pinMode(SWITCH2, INPUT_PULLUP); // Use internal pull-up resistor
  // pinMode(BACK_LIMIT_SWITCH, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(SWITCH1), handleFrontSwitch, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCH2), handleBackSwitch, FALLING);

  Serial.println("Reading in setup ");
  Serial.println(digitalRead(SWITCH1));

  Serial.println("Reading in setup ");
  Serial.println(digitalRead(SWITCH2));
}

void loop() {
  if (!hit_front) {
    Serial.println("HIT FRONT");
    hit_front=!hit_front;
  }
  if (!hit_back) {
    Serial.println("HIT BACK");
    hit_back=!hit_back;
  }
}
