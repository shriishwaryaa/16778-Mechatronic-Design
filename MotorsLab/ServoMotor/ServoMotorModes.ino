#include <Servo.h>

#define SERVO_PIN 9
#define MODE_SWITCH 12
#define SERVO_CONTROL_SWITCH 8
#define LED 7
#define DEBOUNCE_DELAY 50

#define ANGLE_0 0
#define ANGLE_45 45
#define ANGLE_180 180

enum ServoDirection {
  CLOCKWISE = 0,
  COUNTER_CLOCKWISE = 1
} ;

Servo Servo1;

enum OperatingMode {
  SERVO_PUSH_BUTTON = 0,
  SERVO_ACCELEROMETER = 1
} ;

int ModeButtonState;            // the current reading from the input pin
int PrevModeButtonState = HIGH;  // the previous reading from the input pin
int CurrentOperatingMode = SERVO_PUSH_BUTTON;
int CurrentServoPosition;
int CurrentServoDirection;

int ServoChangeModeButtonState;
int PrevServoChangeModeButtonState = HIGH;
unsigned long LastPosDebounceTime = 0;

unsigned long LastDebounceTime = 0;  // the last time the output pin was toggled
bool ServoStepControlMode = false;

int LedState = LOW;

void setup() {
  Serial.begin(9600);

  Servo1.attach(SERVO_PIN);
  pinMode(MODE_SWITCH, INPUT);
  pinMode(SERVO_CONTROL_SWITCH, INPUT);

  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);
  CurrentServoPosition = 0;
  Servo1.write(CurrentServoPosition);

  CurrentServoDirection = CLOCKWISE;
}


int CheckMode() {
  volatile int ModeButtonReading = digitalRead(MODE_SWITCH);
  if (ModeButtonReading != PrevModeButtonState) {
    LastDebounceTime = millis();
  }

  if ((millis() - LastDebounceTime) > DEBOUNCE_DELAY) {
    if (ModeButtonReading != ModeButtonState) {
      ModeButtonState = ModeButtonReading;
      if (ModeButtonState == LOW) {
        switch (CurrentOperatingMode) {
          case SERVO_PUSH_BUTTON:
            Serial.println("Switching to Accelerometer Mode!");
            CurrentOperatingMode = SERVO_ACCELEROMETER;
            break;
          case SERVO_ACCELEROMETER:
            Serial.println("Switching to Push Button Mode!");
            CurrentOperatingMode = SERVO_PUSH_BUTTON;
            break;
          default:
            Serial.println("Something went wrong :(");
            break;
        }
        LedState = !LedState;
        digitalWrite(LED, LedState);
      }
    }
  }
  PrevModeButtonState = ModeButtonReading;
}

void RunAccelerometer() {

}

bool ChangePosition () {
  bool res = false;
  volatile int PositionButtonReading = digitalRead(SERVO_CONTROL_SWITCH);
  if (PositionButtonReading != PrevServoChangeModeButtonState) {
    LastPosDebounceTime = millis();
  }

  if ((millis() - LastPosDebounceTime) > DEBOUNCE_DELAY) {
    if (PositionButtonReading != ServoChangeModeButtonState) {
      ServoChangeModeButtonState = PositionButtonReading;
      if (ServoChangeModeButtonState == LOW) {
        res = true;
      }
    }
  }
  PrevServoChangeModeButtonState = PositionButtonReading;
  return res;
}

void RunServoPushButton() {
  if (CurrentOperatingMode != SERVO_PUSH_BUTTON) {
    return;
  }
  
  int Pressed = ChangePosition();
  if (!Pressed) {
    return;
  } 

  if (CurrentServoDirection == CLOCKWISE) {
    if (CurrentServoPosition == ANGLE_180) {
      CurrentServoDirection = COUNTER_CLOCKWISE;
      CurrentServoPosition -= ANGLE_45;
      Servo1.write(CurrentServoPosition);
      return;
    } else {
      CurrentServoPosition += ANGLE_45;
      Servo1.write(CurrentServoPosition);
      return;
    }
  } else {
    if (CurrentServoPosition == ANGLE_0) {
      CurrentServoDirection = CLOCKWISE;
      CurrentServoPosition += ANGLE_45;
      Servo1.write(CurrentServoPosition);
      return;
    } else {
      CurrentServoPosition -= ANGLE_45;
      Servo1.write(CurrentServoPosition);
      return;
    }
  }
}

void loop() {
  int mode = CheckMode();

  switch (mode) {
    case SERVO_ACCELEROMETER:
      RunAccelerometer();
      break;
    case SERVO_PUSH_BUTTON:
      RunServoPushButton();
      break;
    default:
      Serial.println("Something went wrong :(");
      break;
  }
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


