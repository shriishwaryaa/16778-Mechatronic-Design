#include <Arduino.h>

#define FRONT_LIMIT_SWITCH 2
#define BACK_LIMIT_SWITCH 3

#define SWITCH_DEBOUNCE_TIME 50 // Debounce time in milliseconds

volatile bool hit_front = false;
volatile bool hit_back = false;

unsigned long last_interrupt_time_front = 0;
unsigned long last_interrupt_time_back = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(FRONT_LIMIT_SWITCH, INPUT_PULLUP); // Use internal pull-up resistor
  pinMode(BACK_LIMIT_SWITCH, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(FRONT_LIMIT_SWITCH), handleFrontSwitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_LIMIT_SWITCH), handleBackSwitch, CHANGE);
}

void loop() {
  if (hit_front) {
    Serial.println("Front Limit Switch Triggered!");
    hit_front = false; // Reset the hit flag after handling
  }
  
  if (hit_back) {
    Serial.println("Back Limit Switch Triggered!");
    hit_back = false; // Reset the hit flag after handling
  }

  Serial.println("Boring................");
}

void handleFrontSwitch() {
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
