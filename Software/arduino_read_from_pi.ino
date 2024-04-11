/* Example sketch to control a stepper motor with TB6600 stepper motor driver 
  and Arduino without a library: continuous rotation. 
  More info: https://www.makerguides.com */

// Remember that gearbox ratio is 1:40
// One rev of the stepper is 3200 steps
// 360 degrees_stepper = 3200 steps
// 360 degrees_gearbox = 3200 * 40 steps

// therefore, say 22.5 degrees on gearbox: 360/16 = 3200 * 40 / 16

// Define stepper motor connections:
// L1
#define l1pitchDirPin 2
#define l1pitchStepPin 3

#define l1yawDirPin 8
#define l1yawStepPin 9

// #define STEPS_PER_REV_STEPPER 3200
// Configuration on driver - 1 0 0 0 1 0 :DDDDDDD - Note - with 800 steps :))))))

#define STEPS_PER_REV_GEARBOX 32000

int stepPin = -1;
int dirPin = -1;

bool done = false;

String data = "";
String angleData = "";
String motorData = "";

void setup() {
  // Declare pins as output:
  // L1
  pinMode(l1pitchDirPin, OUTPUT);
  pinMode(l1pitchStepPin, OUTPUT);
  pinMode(l1yawDirPin, OUTPUT);
  pinMode(l1yawStepPin, OUTPUT);

  //L2
  // pinMode(l2pitchDirPin, OUTPUT);
  // pinMode(l2pitchStepPin, OUTPUT);
  // pinMode(l2yawDirPin, OUTPUT);
  // pinMode(l2yawStepPin, OUTPUT);
  Serial.begin(9600);

  // Set the spinning direction CW/CCW:
  digitalWrite(l1yawDirPin, HIGH);
  digitalWrite(l1pitchDirPin, HIGH);

  // digitalWrite(l2yawDirPin, HIGH);
  // digitalWrite(l2pitchDirPin, HIGH);
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
    stepPin = l1pitchStepPin;
    dirPin = l1pitchDirPin;
  }
  else {
    stepPin = l1yawStepPin;
    dirPin = l1yawDirPin;
  }
  // else if (motor == 2) {
  //   stepPin = l2pitchStepPin;
  //   dirPin = l2pitchDirPin;
  // }
  // else {
  //   stepPin = l2yawStepPin;
  //   dirPin = l2yawDirPin;
  // }
  
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
