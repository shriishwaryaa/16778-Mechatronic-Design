#include <Arduino.h>
#define SWITCH 2
#define SWITCH_DEBOUNCE_TIME 50

bool hit_front = true;
unsigned long last_interrupt_time_front = 0;

void handleFrontSwitch() {
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
  pinMode(SWITCH, INPUT_PULLUP); // Use internal pull-up resistor
  // pinMode(BACK_LIMIT_SWITCH, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(SWITCH), handleFrontSwitch, FALLING);

  Serial.println("Reading in setup ");
  Serial.println(digitalRead(SWITCH));
}

void loop() {
  if (!hit_front) {
    Serial.println("HITTT");
    hit_front=!hit_front;
  }
}
