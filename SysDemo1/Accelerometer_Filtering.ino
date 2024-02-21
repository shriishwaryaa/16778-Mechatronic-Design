/*
===Contact & Support===
Website: http://eeenthusiast.com/
Youtube: https://www.youtube.com/EEEnthusiast
Facebook: https://www.facebook.com/EEEnthusiast/
Patreon: https://www.patreon.com/EE_Enthusiast
Revision: 1.0 (July 13th, 2016)

===Hardware===
- Arduino Uno R3
- MPU-6050 (Available from: http://eeenthusiast.com/product/6dof-mpu-6050-accelerometer-gyroscope-temperature/)

===Software===
- Latest Software: https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
- Arduino IDE v1.6.9
- Arduino Wire library

===Terms of use===
The software is provided by EEEnthusiast without warranty of any kind. In no event shall the authors or 
copyright holders be liable for any claim, damages or other liability, whether in an action of contract, 
tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in 
the software.
*/

#include <Wire.h>
#include <Servo.h>

#define ALPHA 0.956
#define T_SAMPLE 0.0002 // 0.2ms
#define SERVO_PIN 9

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float last_gyro_x = 0;
float last_gyro_y = 0;
float last_gyro_z = 0;

Servo Servo1;

void setup() {
  Serial.begin(9600);
  Servo1.attach(SERVO_PIN);
  Wire.begin();
  setupMPU();
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  delay(100);
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

void recordAccelRegisters() {
  // Serial.print("Entering Acc at ");
  // Serial.println(micros());
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
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

  Serial.println(gForceX);
  Serial.println(gForceY);
  Serial.println(gForceZ);
}

void recordGyroRegisters() {
  // Serial.print("Entering Gyro at ");
  // Serial.println(micros());

  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
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

  Serial.print("Angle X ");
  Serial.println(angle_x);

  Serial.print("Angle Y ");
  Serial.println(angle_y);

  Serial.print("Angle Z ");
  Serial.println(angle_z);

  float servo_angle = map(angle_y, -3.2, 3.2, 0, 180);
  Servo1.write(servo_angle);
  
  delay(500);

  // Serial.print("Gyro (deg)");
  // Serial.print(" X=");
  // Serial.print(rotX);
  // Serial.print(" Y=");
  // Serial.print(rotY);
  // Serial.print(" Z=");
  // Serial.print(rotZ);
  // Serial.print(" Accel (g)");
  // Serial.print(" X=");
  // Serial.print(gForceX);
  // Serial.print(" Y=");
  // Serial.print(gForceY);
  // Serial.print(" Z=");
  // Serial.println(gForceZ);
}

