#include <Wire.h>
#define WEIGHT_SCALE_FACTOR 0.4
#define ALPHA 0.956
#define T_SAMPLE 0.0002 // 0.2ms

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float last_gyro_x = 0;
float last_gyro_y = 0;
float last_gyro_z = 0;

int initial1 = 0;
int initial2 = 0;

const int trigPin = 9;
const int echoPin = 10;

float duration, distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(12, OUTPUT);
  initial1 = analogRead(A5);
  initial2 = analogRead(A4);
  Serial.print(initial1);
//  Serial.print(", ");
//  Serial.println(initial2);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize the MPU6050
  Wire.begin();
  Serial.print("pewpew");
  setupMPU();
  Serial.println("OK");
}

bool get_distance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
//  Serial.println(distance);
  if (distance < 30){
    return true;
  }
  else{
    return false;
  }
}

float get_weight(int in){
  int val = map(in, 735, 1023, 0, 5000);
  float R = 5000 - val;
  R *= 10000;
  R /= val;

  float C = 1000000;
  C /= R;

  float F;

//  Serial.print("IN: ");
//  Serial.print(in);
//  Serial.print(", val: ");
//  Serial.print(val);
//  Serial.print(", R: ");
//  Serial.print(R);
//  Serial.print(", C: ");
//  Serial.println(C);

  F = C / 7;
  
  return F;
  
}

bool get_leg_press(int in){
  int val = map(in, 0, 1023, 0, 5000);
//  Serial.println(val);
  if (val > 4000){
    return true;
  }
  return false;
}

void loop() {
  // put your main code here, to run repeatedly:

  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  
  int in1 = analogRead(A5);
  int in2 = analogRead(A4);
  //Serial.print(in1);
  float weight = get_weight(in1);
  bool leg_press = get_leg_press(in2);
  bool d = get_distance();
  Serial.print("Rider weighs: ");
  Serial.print(weight);
  Serial.println("N");
  if (!d){
//    Serial.println("YEE");
    digitalWrite(12, LOW);
  }
  else{
//    Serial.println("HAW");
    digitalWrite(12, HIGH);
  }
  if (!leg_press){
//    Serial.println("Leg Pressure NORMAL");
    digitalWrite(7, LOW);
  }
  else if (leg_press){
//    Serial.println("Leg Pressure TOO HIGH");
    digitalWrite(7, HIGH);
  }
  else{
    Serial.println("I hate myself");
  }
  delay(10);

}

void setupMPU(){
  Wire.begin();
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
//  Serial.print("1");
  Wire.endTransmission();  
//  Serial.print("2");
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Serial.print("3");
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
//  Serial.print("4");
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
//  Serial.print("5");
  Wire.endTransmission(); 
//  Serial.print("6");
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Serial.print("7");
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
//  Serial.print("8");
  Wire.write(0b00000000); //Setting the accel to +/- 2g
//  Serial.print("9");
  Wire.endTransmission(); 
//  Serial.print("10");
}

void recordAccelRegisters() {
  // Serial.print("Entering Acc at ");
  // Serial.println(micros());
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
//  Serial.print(Wire.available());
//  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
  // Serial.print("Leaving Acc at ");
  // Serial.println(micros());
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;

  // Serial.println(gForceX);
  // Serial.println(gForceY);
  // Serial.println(gForceZ);
}

void recordGyroRegisters() {
  // Serial.print("Entering Gyro at ");
  // Serial.println(micros());

  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
//  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
  // Serial.print("Leaving Gyro at ");
  // Serial.println(micros());
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void printData() {
  float accel_angle_x = atan(gForceY/sqrt(gForceX*gForceX+gForceZ*gForceZ))*1/(3.142/180);
  float accel_angle_y = -atan(gForceX/sqrt(gForceY*gForceY+gForceZ*gForceZ))*1/(3.142/180);

  float gyro_x = last_gyro_x + T_SAMPLE * rotX;
  float gyro_y = last_gyro_y + T_SAMPLE * rotY;
  float gyro_z = last_gyro_z + T_SAMPLE * rotZ;

  float angle_x = ALPHA * gyro_x + (1.0 - ALPHA) * accel_angle_x;
  float angle_y = ALPHA * gyro_y + (1.0 - ALPHA) * accel_angle_y;
  float angle_z = gyro_z;  //Accelerometer doesn't give z-angle

//  Serial.print("X: ");
//  Serial.print(angle_x);
//  Serial.print(", Y: ");
//  Serial.println(angle_y);

  if (abs(angle_x) > 0.5 or abs(angle_y) > 0.5){
    digitalWrite(2, HIGH);
  }
  else{
    digitalWrite(2, LOW);
  }

//  if (abs(angle_y) > 1.5){
//    digitalWrite(2, HIGH);
//  }
//  else{
//    digitalWrite(2, LOW);
//  }
  

//  float servo_angle = map(angle_y, -3.2, 3.2, 0, 180);
//  Serial.println(servo_angle);
//  Servo1.write(servo_angle);
  
  delay(500);
}
