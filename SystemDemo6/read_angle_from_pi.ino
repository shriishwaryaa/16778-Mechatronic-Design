/* Example sketch to control a stepper motor with TB6600 stepper motor driver 
  and Arduino without a library: continuous rotation. 
  More info: https://www.makerguides.com */

// Remember that gearbox ratio is 1:40
// One rev of the stepper is 3200 steps
// 360 degrees_stepper = 3200 steps
// 360 degrees_gearbox = 3200 * 40 steps

// therefore, say 22.5 degrees on gearbox: 360/16 = 3200 * 40 / 16

// Define stepper motor connections:
#define dirPin 2
#define stepPin 3

#define STEPS_PER_REV_STEPPER 3200

#define STEPS_PER_REV_GEARBOX 128000

bool done = false;

String data = "";

void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  Serial.begin(9600);

  // Set the spinning direction CW/CCW:
  digitalWrite(dirPin, HIGH);
}

void loop() {
  // read single angle from raspberry pi
  while (Serial.available() == 0) {
    // Serial.println("Waiting for angle input");
  }
  data = Serial.readStringUntil('\n');

  
  float angle = data.toFloat();
  int div = 360 / angle;
  int total_steps = STEPS_PER_REV_GEARBOX / div;

  for (int i = 0; i < total_steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  Serial.println("ok");
  delay(1000);
}
