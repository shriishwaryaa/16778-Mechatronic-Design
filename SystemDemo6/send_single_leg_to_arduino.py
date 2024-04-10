/* Example sketch to control a stepper motor with TB6600 stepper motor driver 
  and Arduino without a library: continuous rotation. 
  More info: https://www.makerguides.com */

// Remember that gearbox ratio is 1:40
// One rev of the stepper is 3200 steps
// 360 degrees_stepper = 3200 steps
// 360 degrees_gearbox = 3200 * 40 steps

// therefore, say 22.5 degrees on gearbox: 360/16 = 3200 * 40 / 16

// Define stepper motor connections:
#define pitchDirPin 2
#define pitchStepPin 3
#define yawDirPin 5
#define yawStepPin 6

#define STEPS_PER_REV_STEPPER 3200

#define STEPS_PER_REV_GEARBOX 128000

int stepPin = -1;
int dirPin = -1;

bool done = false;

String data = "";
String angleData = "";
String motorData = "";

void setup() {
  // Declare pins as output:
  pinMode(pitchDirPin, OUTPUT);
  pinMode(pitchStepPin, OUTPUT);
  pinMode(yawDirPin, OUTPUT);
  pinMode(yawStepPin, OUTPUT);
  Serial.begin(9600);

  // Set the spinning direction CW/CCW:
  digitalWrite(yawDirPin, HIGH);
  digitalWrite(pitchDirPin, HIGH);
}

void loop() {
  
  // read single angle from raspberry pi
  while (Serial.available() == 0) {
    
  }
  // Serial.println(data);
  data = Serial.readStringUntil('\n');
  angleData = data.substring(0, data.indexOf('$'));
  motorData = data.substring(data.indexOf('$') + 1);
  
  float angle = angleData.toFloat();
  float motor = motorData.toFloat();

  if (motor == 0) {
    stepPin = pitchStepPin;
    dirPin = pitchDirPin;
  }
  else {
    stepPin = yawStepPin;
    dirPin = yawDirPin;
  }
  
  if (angle < 0) {
    digitalWrite(dirPin, LOW);
  }
  else {
    digitalWrite(dirPin, HIGH);
  }

  int div = 360 / abs(angle);
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
