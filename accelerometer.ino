/*

*/
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

int x, y, z;
float ax, ay, az;
float axOffset = 0, ayOffset = 0, azOffset = 0;
float axFiltered = 0, ayFiltered = 0, azFiltered = 0;

float alpha = 0.1;
float gravity = 9.81;

// Sensitivity scale factor from datasheet
float sensitivity = 16384.0;  

void setup() {
  Serial.begin(9600);

  // Initialize the MPU6050
  Wire.begin();
  mpu.initialize();

  // Calibrate the MPU6050
  calibrateMPU();
}

void loop() {
  // Read accelerometer data
  mpu.getAcceleration(&x, &y, &z);

  ax = x * gravity / sensitivity - axOffset;
  ay = y * gravity / sensitivity - ayOffset;
  az = z * gravity / sensitivity - azOffset;

  axFiltered = alpha * ax + (1 - alpha) * axFiltered;
  ayFiltered = alpha * ay + (1 - alpha) * ayFiltered;
  azFiltered = alpha * az + (1 - alpha) * azFiltered;

  // Print the values
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(ax);
  Serial.print(" | Y = "); Serial.print(ay);
  Serial.print(" | Z = "); Serial.println(az);

  delay(500);
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050. Please keep the sensor stationary.");

  int numSamples = 1000;
  int tx, ty, tz;
  long xSum = 0, ySum = 0, zSum = 0;

  for (int i = 0; i < numSamples; i++) {
    mpu.getAcceleration(&tx, &ty, &tz);

    xSum += tx;
    ySum += ty;
    zSum += tz;

    delay(5);
  }

  axOffset = (xSum / numSamples) * gravity / sensitivity;
  ayOffset = (ySum / numSamples) * gravity / sensitivity;
  azOffset = (zSum / numSamples) * gravity / sensitivity - gravity;

  Serial.println("Calibration complete.");
  Serial.print("X Offset: "); Serial.println(axOffset);
  Serial.print("Y Offset: "); Serial.println(ayOffset);
  Serial.print("Z Offset: "); Serial.println(azOffset);
}

