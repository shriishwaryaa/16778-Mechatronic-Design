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
