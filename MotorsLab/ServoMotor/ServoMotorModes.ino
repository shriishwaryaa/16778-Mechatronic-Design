#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

#define SERVO_PIN 9
#define MODE_SWITCH 12
#define SERVO_CONTROL_SWITCH 8
#define LED 7
#define DEBOUNCE_DELAY 50

#define NUM_SAMPLES 1000
#define ACCELEROMETER_FACTOR 16384.0
#define GYRO_FACTOR 131.0
#define ALPHA 0.1
#define A_GRAVITY 9.81

#define ANGLE_0 0
#define ANGLE_45 45
#define ANGLE_180 180

enum ServoDirection {
  CLOCKWISE = 0,
  COUNTER_CLOCKWISE = 1
} ;

Servo Servo1;
MPU6050 mpu;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

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

float axOffset = 0, ayOffset = 0, azOffset = 0;

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

  Wire.begin();
  mpu.initialize();

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

void CalibrateMPU() {
  Serial.println("Calibrating MPU6050. Please keep the sensor stationary.");
  int tx, ty, tz;
  long xSum = 0, ySum = 0, zSum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    mpu.getAcceleration(&tx, &ty, &tz);

    xSum += tx;
    ySum += ty;
    zSum += tz;

    delay(5);
  }

  axOffset = (xSum / NUM_SAMPLES) * A_GRAVITY / ACCELEROMETER_FACTOR;
  ayOffset = (ySum / NUM_SAMPLES) * A_GRAVITY / ACCELEROMETER_FACTOR;
  azOffset = (zSum / NUM_SAMPLES) * A_GRAVITY / ACCELEROMETER_FACTOR - A_GRAVITY;

  Serial.println("Calibration complete.");
  Serial.print("X Offset: "); Serial.println(axOffset);
  Serial.print("Y Offset: "); Serial.println(ayOffset);
  Serial.print("Z Offset: "); Serial.println(azOffset);
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void ProcessAccelData(){
  gForceX = accelX / ACCELEROMETER_FACTOR;
  gForceY = accelY / ACCELEROMETER_FACTOR; 
  gForceZ = accelZ / ACCELEROMETER_FACTOR;
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  ProcessAccelData();
}

void RecordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  ProcessGyroData();
}

void ProcessGyroData() {
  rotX = gyroX / GYRO_FACTOR;
  rotY = gyroY / GYRO_FACTOR; 
  rotZ = gyroZ / GYRO_FACTOR;
}

void PrintAccelerometerData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
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


