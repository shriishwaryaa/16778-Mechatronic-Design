// Ultrasonic sensor code adapted from: 
// https://projecthub.arduino.cc/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-7cabe1
#include <Wire.h>
#include <MPU6050.h>

#define TRIG_PIN 9
#define ECHO_PIN 10
#define DIST_LED 8
#define FORCE_SENSOR_PIN A0
#define WEIGHT_A 0.4
#define SENSITIIVITY 16384.0
#define ALPHA 0.1
#define A_GRAVITY 9.81

int filteredValue;
float axOffset = 0, ayOffset = 0, azOffset = 0;

MPU6050 mpu;

void setup() {
  // Initialize the serial monitor
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(DIST_LED, OUTPUT);

  filteredValue = 0;

  // Initialize the MPU6050
  Wire.begin();
  mpu.initialize();

  // Calibrate the Accelerometer
  calibrateMPU();
}

void loop() {
  Serial.println("Running ultrasonic sensor function....");
  ultrasonic_sensor();

  Serial.println("Running force sensitive resistor function....");
  force_sensitive_resistor();

  Serial.println("Running the accelerometer function....");
  accelerometer();
}

void ultrasonic_sensor() {
  float distance = 0.0;
  float duration = 0.0;

  // Initially set the Trig Pin low for 2 microseconds
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Start transmitting the sonic beam
  digitalWrite(TRIG_PIN, HIGH);
  // Transmit for 10 microseconds
  delayMicroseconds(10);
  // Stop transmitting the sonic beam
  digitalWrite(TRIG_PIN, LOW);

  // Check the time for which the echo pin was high
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration*.0343)/2;
  
  if (distance < 2 || distance > 400)
  {
    Serial.println("Distance Out of Range");
  }
  else
  {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if(distance < 20)
    {
      digitalWrite(DIST_LED, HIGH);   
    }
    else
    {
      digitalWrite(DIST_LED, LOW);
    }
  }
  delay(100);
}

void force_sensitive_resistor() {
  int fsrValue = 0;

  fsrValue = analogRead(FORCE_SENSOR_PIN);

  Serial.print("Force sensor reading = ");
  Serial.println(fsrValue);
  
  if (filteredValue == 0) {
    filteredValue = fsrValue;
  }
  
  filteredValue = WEIGHT_A * fsrValue + (1 - WEIGHT_A) * filteredValue;
  
  // Print the filtered value
  Serial.print(", Filtered Value = ");
  Serial.println(filteredValue);

  if (filteredValue < 10)
    Serial.println(" -> No Pressure Applied");
  else if (filteredValue < 200)
    Serial.println(" -> Light Pressure Applied");
  else if (filteredValue < 500)
    Serial.println(" -> Light Squeeze Applied");
  else if (filteredValue < 800)
    Serial.println(" -> Medium Squeeze Applied");
  else
    Serial.println(" -> Large Pressure Applied...seeks help");

  delay(1000);
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

  axOffset = (xSum / numSamples) * A_GRAVITY / SENSITIIVITY;
  ayOffset = (ySum / numSamples) * A_GRAVITY / SENSITIIVITY;
  azOffset = (zSum / numSamples) * A_GRAVITY / SENSITIIVITY - A_GRAVITY;

  Serial.println("Calibration complete.");
  Serial.print("X Offset: "); Serial.println(axOffset);
  Serial.print("Y Offset: "); Serial.println(ayOffset);
  Serial.print("Z Offset: "); Serial.println(azOffset);
}

void accelerometer() {
  float ax, ay, az;
  float axFiltered = 0.0, ayFiltered = 0.0, azFiltered = 0.0;
  int x, y, z;
  
  // Read accelerometer data
  mpu.getAcceleration(&x, &y, &z);

  ax = x * A_GRAVITY / SENSITIIVITY - axOffset;
  ay = y * A_GRAVITY / SENSITIIVITY - ayOffset;
  az = z * A_GRAVITY / SENSITIIVITY - azOffset;

  axFiltered = ALPHA * ax + (1 - ALPHA) * axFiltered;
  ayFiltered = ALPHA * ay + (1 - ALPHA) * ayFiltered;
  azFiltered = ALPHA * az + (1 - ALPHA) * azFiltered;

  // Print the values
  Serial.print("Accelerometer Readings: ");
  Serial.print("X = "); Serial.print(ax);
  Serial.print(" | Y = "); Serial.print(ay);
  Serial.print(" | Z = "); Serial.println(az);

  delay(500);
}