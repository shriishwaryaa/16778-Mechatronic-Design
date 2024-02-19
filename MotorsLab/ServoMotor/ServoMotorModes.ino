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
#define ACCELEROMETER_FACTOR_2G 16384.0
#define ACCELEROMETER_FACTOR_4G 8192.0
#define ACCELEROMETER_FACTOR_6G 4096.0
#define ACCELEROMETER_FACTOR_8G 2048.0

#define GYRO_FACTOR 131.0
#define ALPHA 0.1
#define A_GRAVITY 9.81
#define MAGNETOMETER_CAL 0.06
#define MAGNETOMETER_CONSTANT 0.6

#define ANGLE_0 0
#define ANGLE_45 45
#define ANGLE_180 180

enum ServoDirection {
  CLOCKWISE = 0,
  COUNTER_CLOCKWISE = 1
} ;

enum OperatingMode {
  SERVO_PUSH_BUTTON = 0,
  SERVO_ACCELEROMETER = 1
} ;

Servo Servo1;
MPU9255 mpu;

int ModeButtonState;              // Current state of mode button         
int PrevModeButtonState = HIGH;  // Previous state of mode button     
int CurrentOperatingMode = SERVO_PUSH_BUTTON; // System starts in Push Button mode
int CurrentServoPosition;       // Current angular position of servo motor
int CurrentServoDirection;      // Clockwise or anticlockwise rotation
int ServoChangeModeButtonState; // Current state of servo position button 
int PrevServoChangeModeButtonState = HIGH; // Previous state of servo position button 
unsigned long LastPosDebounceTime = 0; // The last instance of time the servo position switch was toggled
unsigned long LastDebounceTime = 0;  // The last instance of time the mode switch was toggled
bool ServoStepControlMode = false; // Flag to indicate mode of servo motor control
unsigned long LastProgramLoop = 0.0;

int LedState = LOW; // Mode indicator LED state

void setup() {
  Serial.begin(9600);

  Servo1.attach(SERVO_PIN); // Servo attached to Pin 9
  pinMode(MODE_SWITCH, INPUT); // Input switch to control current working mode
  pinMode(SERVO_CONTROL_SWITCH, INPUT); // Switch to control servo position
  pinMode(LED, OUTPUT); // LED which indicates current mode

  digitalWrite(LED, LOW); 
  CurrentServoPosition = 0;
  Servo1.write(CurrentServoPosition);

  CurrentServoDirection = CLOCKWISE;
  // Initialize MPU
  Wire.begin();
  mpu.init();
}

// Checks the current operating mode of the system and returns the same
int CheckMode() {
  volatile int ModeButtonReading = digitalRead(MODE_SWITCH); // Fetch the current reading of the mode button

  if (ModeButtonReading != PrevModeButtonState) {
    // If the state of the mode button has changed, start the debounce timer
    LastDebounceTime = millis();
  }

  if ((millis() - LastDebounceTime) > DEBOUNCE_DELAY) {
    // If button was pressed for more than DEBOUNCE_DELAY milliseconds, then take action
    if (ModeButtonReading != ModeButtonState) {
      ModeButtonState = ModeButtonReading;
      if (ModeButtonState == LOW) {
        // We use a pull up resisitor with the push button switch, hence the switch reads LOW when 
        // pressed and HIGH otherwise
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

/*
Processes the acceleration data read from the MPU
Data is read from the 6 registers dedicated for accelerometer data 
via I2C. We divide the readings by the SensorScale as specified in the 
data sheet to get the correct values for accelerometer readings

input = raw reading from the sensor, SensorScale = selected sensor scale
returns : acceleration in units of g
*/

double process_acceleration(int Input, scales SensorScale)
{
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default scale)
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale
  */
  double Output = 1.0;

  // for +- 2g

  if(SensorScale == scale_2g)
  {
    Output = Input / ACCELEROMETER_FACTOR_2G;
    Output = Output * A_GRAVITY;
  }

  // for +- 4g

  if(SensorScale == scale_4g)
  {
    Output = Input / ACCELEROMETER_FACTOR_4G;
    Output = Output * A_GRAVITY;
  }

  // for +- 8g

  if(SensorScale == scale_8g)
  {
    Output = Input / ACCELEROMETER_FACTOR_6G;
    Output = Output * A_GRAVITY;
  }

  // for +-16g

  if(SensorScale == scale_16g)
  {
    Output = Input / ACCELEROMETER_FACTOR_8G;
    Output = Output * A_GRAVITY;
  }

  return Output;
}

/*
Processes the gyroscope data read from the MPU.
Data is read from the 6 registers dedicated for gyroscope data 
via I2C. We divide the readings by the SensorScale as specified in the 
data sheet to get the correct values for gyroscope readings

input = raw reading from the sensor, SensorScale = selected sensor scale
returns : angular velocity in degrees per second
*/

double process_angular_velocity(int16_t Input, scales SensorScale)
{
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */

  // for +- 250 dps

  if(SensorScale == scale_250dps)
  {
    return Input/131.0;
  }

  // for +- 500 dps

  if(SensorScale == scale_500dps)
  {
    return Input/65.5;
  }

  // for +- 1000 dps

  if(SensorScale == scale_1000dps)
  {
    return Input/32.8;
  }

  // for +- 2000 dps

  if(SensorScale == scale_2000dps)
  {
    return Input/16.4;
  }

  return 0;
}

/*
Processes the gyroscope data read from the MPU.
Data is read from the 6 registers dedicated for magnetometer data 
via I2C. We divide the readings by the SensorScale as specified in the 
data sheet to get the correct values for gyroscope readings

Input = raw reading from the sensor, Sensitivity - selected sensitivity
returns : magnetic flux density in μT (in micro Teslas)
*/

double process_magnetic_flux(int16_t Input, double Sensitivity)
{
  /*
  To get magnetic flux density in μT, each reading has to be multiplied by sensitivity
  (Constant value different for each axis, stored in ROM), then multiplied by some number (calibration)
  and then divided by 0.6 .
  (Faced North each axis should output around 31 µT without any metal / walls around
  Note : This magnetometer has really low initial calibration tolerance : +- 500 LSB 
  Scale of the magnetometer is fixed -> +- 4800 μT.
  */
  return (Input * MAGNETOMETER_CAL * Sensitivity) / MAGNETOMETER_CONSTANT;
}

void RunAccelerometer() {
  if (CurrentOperatingMode != SERVO_ACCELEROMETER) {
    return;
  }

  mpu.read_acc(); // get data from the accelerometer
  mpu.read_gyro(); // get data from the gyroscope
  mpu.read_mag(); // get data from the magnetometer

  double AZServo = 0.0;

  // process and print acceleration data
  // X axis
  Serial.print("AX: ");
  Serial.print(process_acceleration(mpu.ax,scale_2g));

  // Y axis
  Serial.print("  AY: ");
  Serial.print(process_acceleration(mpu.ay,scale_2g));

  // Z axis
  Serial.print("  AZ: ");
  AZServo = process_acceleration(mpu.az,scale_2g);
  Serial.print(AZServo);

  // process and print gyroscope data
  // X axis
  Serial.print("      GX: ");
  Serial.print(process_angular_velocity(mpu.gx,scale_250dps));

  // Y axis
  Serial.print("  GY: ");
  Serial.print(process_angular_velocity(mpu.gy,scale_250dps));

  // Z axis
  Serial.print("  GZ: ");
  Serial.print(process_angular_velocity(mpu.gz,scale_250dps));


  // process and print magnetometer data
  // X axis
  Serial.print("      MX: ");
  Serial.print(process_magnetic_flux(mpu.mx,mpu.mx_sensitivity));

  // Y axis
  Serial.print("  MY: ");
  Serial.print(process_magnetic_flux(mpu.my,mpu.my_sensitivity));

  // Z axis
  Serial.print("  MZ: ");
  Serial.println(process_magnetic_flux(mpu.mz,mpu.mz_sensitivity));

  // Maps the Z-axis acceleration to an angle for the servo motor

  double ServoMapAngle = map (AZServo, -9.85, 9.85, 0, 180) ;
  Serial.print("Writing ");
  Serial.print(ServoMapAngle);
  Serial.println(" to servo");
  Servo1.write(ServoMapAngle);

  delay(500);
}

/*
  Reads from the push button which changes the servo motor positions. Uses a debouncing
  Mechanism to detect pin state change. Returns true if button was pressed and the servo
  position must be changed, false otherwise.
*/

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

/*
  Contains functionality to be performed when the system is in the mode
  which controls the servo motor using input from the push button
*/

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


