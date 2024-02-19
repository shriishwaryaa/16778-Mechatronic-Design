#include <Servo.h>

#define SERVO_PIN 9
#define MODE_SWITCH 12
#define SERVO_CONTROL_SWITCH 3
#define LED 7
#define DEBOUNCE_DELAY 50

Servo Servo1;

int ModeButtonState;            // the current reading from the input pin
int PrevModeButtonState = HIGH;  // the previous reading from the input pin

unsigned long LastDebounceTime = 0;  // the last time the output pin was toggled
bool ServoStepControlMode = false;

int LedState = LOW;

void setup() {
  Serial.begin(9600);

  Servo1.attach(SERVO_PIN);
  pinMode(MODE_SWITCH, INPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);
}

void Actuate_Servo () {
  Servo1.write(0);
  delay(1000);
  Servo1.write(90);
  delay(500);
  Servo1.write(135);
  delay(500);
  Servo1.write(180);
  delay(1500);
}

void loop() {
  volatile int ModeButtonReading = digitalRead(MODE_SWITCH);
  if (ModeButtonReading != PrevModeButtonState) {
    LastDebounceTime = millis();
  }

  if ((millis() - LastDebounceTime) > DEBOUNCE_DELAY) {
    if (ModeButtonReading != ModeButtonState) {
      ModeButtonState = ModeButtonReading;
      if (ModeButtonState == LOW) {
        Serial.println("Changing Mode!");
        LedState = !LedState;
        digitalWrite(LED, LedState);
      }
    }
  }
  PrevModeButtonState = ModeButtonReading;
}

/*
  created 21 Nov 2006
  by David A. Mellis
  modified 30 Aug 2011
  by Limor Fried
  modified 28 Dec 2012
  by Mike Walters
  modified 30 Aug 2016
  by Arturo Guadalupi

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Debounce
*/


