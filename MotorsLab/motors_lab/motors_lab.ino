/*
MPU9255 Library - https://github.com/Bill2462/MPU9255-Arduino-Library
Debounce Login - https://www.arduino.cc/en/Tutorial/BuiltInExamples/Debounce
*/

#include <Servo.h>
#include <Wire.h>
#include <MPU9255.h>
#include <Stepper.h>

int system_state = 5;

int gui_dc_speed = 0;
int gui_dc_pos = 0;
int gui_servo_pos = 0;
int gui_stepper_count = 0;
int gui_ultrasonic = 0;
int gui_temp = 0;
int gui_pot1 = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////

#define SERVO_PIN 9
#define MODE_SWITCH 12
#define SERVO_CONTROL_SWITCH 8
#define LED 7
#define DEBOUNCE_DELAY 25

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

/////////////////////////////////////////////////////////////////////////////////////////////////////

#define ENCA 32 //Blue wire; encoder A
#define ENCB 34 // Purple wire; encoder B
#define PWM 6 // Enable pin capable of PWM
#define IN1 4 // Enable Dir1 pin
#define IN2 5 //Enable Dir2 pin
#define FORCE_SENSOR_PIN A0

const int potPin = A1; //Potentiometer
volatile int32_t motor_posn = 0;

int motor_speed, mapped_speed;
float target = 0;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float v3Filt = 0;
float v3Prev = 0;
float v3 = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////

const int stepsPerRevolution = 2038;
const int trigPin = 26;
const int echoPin = 28;
const int threshold = 10;
float duration, distance;
int stepCount = 0;
// unsigned long lastDebounceTimeStep = 0;  // the last time the output pin was toggled
// unsigned long debounceDelayStep = 50;    // the debounce time; increase if the output flickers
// int lastButtonState = LOW;
// int buttonState; 
// int Steps; 

// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 13, 11, 3, 2);

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

  /////////////////////////////////////////////////////////////////////////////////////////////////////

  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);

  pinMode(PWM, OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

  /////////////////////////////////////////////////////////////////////////////////////////////////////

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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
            // Serial.println("Switching to Accelerometer Mode!");
            CurrentOperatingMode = SERVO_ACCELEROMETER;
            break;
          case SERVO_ACCELEROMETER:
            // Serial.println("Switching to Push Button Mode!");
            CurrentOperatingMode = SERVO_PUSH_BUTTON;
            break;
          default:
            // Serial.println("Something went wrong :(");
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
  // Serial.print("AX: ");
  // Serial.print(process_acceleration(mpu.ax,scale_2g));

  // Y axis
  // Serial.print("  AY: ");
  // Serial.print(process_acceleration(mpu.ay,scale_2g));

  // Z axis
  // Serial.print("  AZ: ");
  AZServo = process_acceleration(mpu.az,scale_2g);
  // Serial.print(AZServo);

  // process and print gyroscope data
  // X axis
  // Serial.print("      GX: ");
  // Serial.print(process_angular_velocity(mpu.gx,scale_250dps));

  // Y axis
  // Serial.print("  GY: ");
  // Serial.print(process_angular_velocity(mpu.gy,scale_250dps));

  // Z axis
  // Serial.print("  GZ: ");
  // Serial.print(process_angular_velocity(mpu.gz,scale_250dps));


  // process and print magnetometer data
  // X axis
  // Serial.print("      MX: ");
  // Serial.print(process_magnetic_flux(mpu.mx,mpu.mx_sensitivity));

  // Y axis
  // Serial.print("  MY: ");
  // Serial.print(process_magnetic_flux(mpu.my,mpu.my_sensitivity));

  // Z axis
  // Serial.print("  MZ: ");
  // Serial.println(process_magnetic_flux(mpu.mz,mpu.mz_sensitivity));

  // Maps the Z-axis acceleration to an angle for the servo motor

  double ServoMapAngle = map (AZServo, -9.85, 9.85, 0, 180) ;
  // Serial.print("Writing ");
  // Serial.print(ServoMapAngle);
  // Serial.println(" to servo");
  Servo1.write(ServoMapAngle);
  gui_servo_pos = ServoMapAngle;

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

  gui_servo_pos = CurrentServoPosition;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void setMotorPosn(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, pwmVal);
  if (dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if (dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
  }
}

void readEncoder(){
  int b = digitalRead(ENCB); //Reads encoder b

  if (system_state == 2) {
    if(b>0){
      motor_posn++;
    }
    else{
      motor_posn--;
    }
  }

  if (system_state == 3) {
    int increment = 0;
    if(b>0){
      increment = 1;
    }
    else{
      increment = -1;
    }
    pos_i = pos_i+increment;
  
    long currT = micros();
    float deltaT = ((float)(currT-prevT_i))/1.0e6;
    velocity_i = increment/deltaT;
    prevT_i = currT;
  }

  if (system_state == 4) {
    int increment = (b > 0) ? 1 : -1;
  
    pos_i = pos_i + increment;
  
    long currT = micros();
    float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
    velocity_i = increment / deltaT;
    prevT_i = currT;
  }
  
}

void setMotorSpeed(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, pwmVal);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  // if (dir == 1){
  //   digitalWrite(in1,HIGH);
  //   digitalWrite(in2,LOW);
  // }
  // else if (dir == -1){
  //   digitalWrite(in1,LOW);
  //   digitalWrite(in2,HIGH);
  // }
  // else{
  //     digitalWrite(in1,LOW);
  //     digitalWrite(in2,LOW);
  // }
}

void dcPositionControl(){
  target = analogRead(potPin);
  gui_pot1 = target;
  target = map(target, 0, 1023, 0, 360);
  // Tunable gains
  float kp = 1.1;
  float kd = 0.01;
  float ki = 0.5;

  //Time difference
  long currT = micros(); //Current time in microseconds

  float deltaT = ((float)(currT-prevT))/1.0e6;
  
  prevT = currT;
  
  int pos;
  noInterrupts();
  pos = motor_posn;

  gui_dc_pos = pos;
  interrupts();
  // Serial.println("Motor posn set initially");
  
  //error
  int e = -target+pos; //Switched this motor will continuously spin if not 

  //derivative
  float dedt = (e-eprev)/(deltaT);
  // Serial.print("Derivative term: ");
  // Serial.println(dedt);

  //integral
  eintegral = eintegral + e*deltaT;

  //control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if (pwr>255){
    pwr = 255;
  }

  //motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  //signal the motor
  setMotorPosn(dir,pwr,PWM, IN1, IN2);

  // store previous error
  // eprev = 0;
  eprev = e;

  // Serial.print("Target Posn: ");
  // Serial.print(target);
  // Serial.print(" ");

  // Serial.print("Motor Posn: ");
  // Serial.println(motor_posn);
}

void dcVelocityControl(){
  float target_speed = analogRead(potPin);
  gui_pot1 = target;
  target_speed = map(target_speed,0,1023,0,255);
  // Serial.println(target_speed);

  int pos = 0;
  float velocity2 = 0;

  noInterrupts();
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts();  

  long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  float velocity1 = (pos-posPrev)/deltaT;
  // float velocity1 = fabs((pos - posPrev) * 60 / (43.8 * deltaT));

  
  float velocity3 = (pos-posPrev)*60/(98*deltaT); //This one is most accurate
  posPrev = pos;
  prevT = currT;
  
  //Convert to RPM
  // float v1 = (velocity1/(108))*60.0; //108
  float v1 = (velocity1/(16*43.8))*60.0;
  // float v2 = (velocity2/600)*60.0;
  float v3 = (velocity3/(16*43.8))*60.0;

  v1Filt = 0.854*v1Filt+0.0728*v1+0.0728*v1Prev;
  v1Prev = v1;

  // v2Filt = 0.854*v2Filt+0.0728*v2+0.0728*v2Prev;
  // v2Prev = v2;

  // Serial.print("v1Filt: ");
  // Serial.println(v1Filt);

  v3Filt = 0.854*v3Filt+0.0728*v3+0.0728*v3Prev; //This is the filter velocity
  v3Prev = v3;

  // Serial.print("v3Filt: ");
  // Serial.println(v3Filt);

  float curr_motor_vel;
  curr_motor_vel = v1Filt;

  gui_dc_speed = curr_motor_vel;
  
  float kp = 1.5; //1.5 
  float ki = 0.025;
  float kd = 0; //0.1
  float target_speed_rpm = target_speed; //(target_speed/100)*60;

  float e = target_speed_rpm - curr_motor_vel;
  // Serial.print("Error: ");
  // Serial.println(e);
  
  eintegral = eintegral+e*deltaT;

  float dedt = (e-eprev)/(deltaT);
  float u = kp*e+kd*dedt+ki*eintegral;
  // Serial.print(u);
  int dir = -1;
  if (u<0){
    dir = 1;
  }

  int pwr = (int) fabs(u);
  if (pwr>255){
    pwr = 255;
  }

  eprev = 0;

  setMotorSpeed(dir,pwr,PWM, IN1, IN2);

  // Serial.print("Target Speed: ");
  // Serial.println(target_speed_rpm);
  // Serial.print("Current Motor Speed: ");
  // Serial.println(curr_motor_vel); 
  delay(500);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void dcSpeedControl() {
  int fsrReading = analogRead(FORCE_SENSOR_PIN);
  
  // If force is detected, stop the motor
  if (fsrReading > 10) {
    setMotor(0, 0, PWM, IN1, IN2); // Stop the motor
    // Serial.println("Force detected. Motor stopped.");
  }

  else{
    setMotor(1, 100, PWM, IN1, IN2);
    // Serial.println("Motor spinning at speed 100");
  }

  int pos = 0;
  float velocity2 = 0;
  noInterrupts();
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); 

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = (velocity1 / 108) * 60.0;
  float v2 = (velocity2 / 210) * 60.0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

bool us_dist(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  gui_ultrasonic = distance;

  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" cm");

  if(distance < threshold)
  {
    //delay(100);
    return true;  
  }
  else
  {
    //delay(100);
    return false;
  }
  
}

void stepperControl() {
  int sensorReading = analogRead(A2);
  int motorSpeed = map(sensorReading, 0, 1023, -10, 10);
  int dir = 1; // change direction based on mapping
  if (motorSpeed < 0) dir = -1;
  //Serial.println(motorSpeed);
  
  bool dist = us_dist();

  if (abs(motorSpeed) > 0 && !dist) {
    myStepper.setSpeed(abs(motorSpeed));
    
    // step 1/100 of a revolution:
    myStepper.step(dir*(stepsPerRevolution / 10));
    stepCount += 1;
    delay(abs(motorSpeed));
  }
  gui_stepper_count = stepCount;
  
        // safe mode
  if (dist) {
    myStepper.setSpeed(0);
  }
}

void loop() {
  // Get system state through GUI
  // setMotor(1, 100, PWM, IN1, IN2); // speed control of DC

  if (Serial.available() > 0) {
    int StringCount = 0;
    String strs[5];

    String str = Serial.readString();
    while (str.length() > 0)
    {
      int index = str.indexOf(' ');
      if (index == -1) // No space found
      {
        strs[StringCount++] = str;
        break;
      }
      else
      {
        strs[StringCount++] = str.substring(0, index);
        str = str.substring(index+1);
      }
    }
    system_state = strs[0].toInt();
  }
  
  // Run servo motor
  if (system_state == 1) {
    int mode = CheckMode();

    switch (mode) {
      case SERVO_ACCELEROMETER:
        RunAccelerometer();
        break;
      case SERVO_PUSH_BUTTON:
        RunServoPushButton();
        break;
      default:
        // Serial.println("Something went wrong :(");
        break;
    }
  }

  // Run DC motor position control
  if (system_state == 2) {
    dcPositionControl();
  }
  
  // Run DC motor velocity control
  if (system_state == 3) {
    eprev = 0;
    eintegral = 0;
    dcVelocityControl();
  }

  // Run DC motor speed control
  if (system_state == 4) {
    dcSpeedControl();
  }

  // Run servo motor
  if (system_state == 5) {
    stepperControl();
  }

  String gui = String(gui_dc_speed) + " " + String(gui_dc_pos) + " " + String(gui_servo_pos) + " " + String(gui_stepper_count) + " " + String(gui_ultrasonic) + " " + String(gui_pot1) + "\n";
  Serial.write(gui.c_str());
}