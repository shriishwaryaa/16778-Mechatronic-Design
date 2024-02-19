/*
MPU9255 Library - https://github.com/Bill2462/MPU9255-Arduino-Library
Debounce Login - https://www.arduino.cc/en/Tutorial/BuiltInExamples/Debounce
*/

#include <Servo.h>
#include <Wire.h>
#include <MPU9255.h>

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
#define MAGNETOMETER_CAL 0.06

#define ANGLE_0 0
#define ANGLE_45 45
#define ANGLE_180 180

enum ServoDirection {
  CLOCKWISE = 0,
  COUNTER_CLOCKWISE = 1
} ;

Servo Servo1;
MPU9255 mpu;

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

  Wire.begin();
  mpu.init();
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
  return CurrentOperatingMode;
}

double process_acceleration(int input, scales sensor_scale )
{
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default scale)
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale
  */
  double output = 1;

  //for +- 2g

  if(sensor_scale == scale_2g)
  {
    output = input;
    output = output/16384;
    output = output*A_GRAVITY;
  }

  //for +- 4g
  if(sensor_scale == scale_4g)
  {
    output = input;
    output = output/8192;
    output = output*A_GRAVITY;
  }

  //for +- 8g
  if(sensor_scale == scale_8g)
  {
    output = input;
    output = output/4096;
    output = output*A_GRAVITY;
  }

  //for +-16g
  if(sensor_scale == scale_16g)
  {
    output = input;
    output = output/2048;
    output = output*A_GRAVITY;
  }

  return output;
}

//process raw gyroscope data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : angular velocity in degrees per second
double process_angular_velocity(int16_t input, scales sensor_scale )
{
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */

  //for +- 250 dps
  if(sensor_scale == scale_250dps)
  {
    return input/131;
  }

  //for +- 500 dps
  if(sensor_scale == scale_500dps)
  {
    return input/65.5;
  }

  //for +- 1000 dps
  if(sensor_scale == scale_1000dps)
  {
    return input/32.8;
  }

  //for +- 2000 dps
  if(sensor_scale == scale_2000dps)
  {
    return input/16.4;
  }

  return 0;
}

//process raw magnetometer data
//input = raw reading from the sensor, sensitivity =
//returns : magnetic flux density in μT (in micro Teslas)
double process_magnetic_flux(int16_t input, double sensitivity)
{
  /*
  To get magnetic flux density in μT, each reading has to be multiplied by sensitivity
  (Constant value different for each axis, stored in ROM), then multiplied by some number (calibration)
  and then divided by 0.6 .
  (Faced North each axis should output around 31 µT without any metal / walls around
  Note : This manetometer has really low initial calibration tolerance : +- 500 LSB !
  Scale of the magnetometer is fixed -> +- 4800 μT.
  */
  return (input*MAGNETOMETER_CAL*sensitivity)/0.6;
}

void RunAccelerometer() {
  if (CurrentOperatingMode != SERVO_ACCELEROMETER) {
    return;
  }

  mpu.read_acc();//get data from the accelerometer
  mpu.read_gyro();//get data from the gyroscope
  mpu.read_mag();//get data from the magnetometer

  double az_servo = 0.0;

  ////process and print acceleration data////
  //X axis
  Serial.print("AX: ");
  Serial.print(process_acceleration(mpu.ax,scale_2g));

  //Y axis
  Serial.print("  AY: ");
  Serial.print(process_acceleration(mpu.ay,scale_2g));

  //Z axis
  Serial.print("  AZ: ");
  az_servo = process_acceleration(mpu.az,scale_2g);

  ////process and print gyroscope data////
  //X axis
  Serial.print("      GX: ");
  Serial.print(process_angular_velocity(mpu.gx,scale_250dps));

  //Y axis
  Serial.print("  GY: ");
  Serial.print(process_angular_velocity(mpu.gy,scale_250dps));

  //Z axis
  Serial.print("  GZ: ");
  Serial.print(process_angular_velocity(mpu.gz,scale_250dps));


  ////process and print magnetometer data////
  //X axis
  Serial.print("      MX: ");
  Serial.print(process_magnetic_flux(mpu.mx,mpu.mx_sensitivity));

  //Y axis
  Serial.print("  MY: ");
  Serial.print(process_magnetic_flux(mpu.my,mpu.my_sensitivity));

  //Z axis
  Serial.print("  MZ: ");
  Serial.println(process_magnetic_flux(mpu.mz,mpu.mz_sensitivity));

  double servo_map_angle = map (az_servo, -9.85, 9.85, 0, 180) ;
  Serial.print("Writing ");
  Serial.print(servo_map_angle);
  Serial.println(" to servo");
  Servo1.write(servo_map_angle);

  delay(500);
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


