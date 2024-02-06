#include <MPU6050.h>
#define FORCE_SENSOR_PIN A0
const float weight_a = 0.4;

int filteredValue = 0;

void setup() {
    Serial.begin(9600);
}

void loop() {
  int fsrValue = analogRead(FORCE_SENSOR_PIN);
  Serial.print("Force sensor reading = ");
  Serial.print(fsrValue);
  
  if (filteredValue == 0) {
    filteredValue = fsrValue;
  }
  
  filteredValue = weight_a * fsrValue + (1 - weight_a) * filteredValue;
  float force_converted = mapToForce(filteredValue);
  Serial.print(", Force = ");
  Serial.println(force_converted);

  // if (force_converted < 10)
  //   Serial.println(" -> No Pressure Applied");
  // else if (force_converted < 30)
  //   Serial.println(" -> Light Pressure Applied");
  // else if (force_converted < 60)
  //   Serial.println(" -> Light Squeeze Applied");
  // else if (force_converted < 80)
  //   Serial.println(" -> Medium Squeeze Applied");
  // else
  //   Serial.println(" -> Large Pressure Applied");

  delay(1000);
}

float mapToForce(int analogReading){
  const int minValue = 10;
  const int maxValue = 1023;

  float force = map(analogReading, minValue, maxValue, 0.1,100);
  return force;
}