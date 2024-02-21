// stops permanently when threshold is met
//Includes the Arduino Stepper Library
#include <Stepper.h>

// Defines the number of steps per rotation
const int stepsPerRevolution = 2038;
const int buttonPin = 2;
const int trigPin = 7;
const int echoPin = 6;
const int threshold = 10;
float duration, distance;
int stepCount = 0;
unsigned long lastDebounceTimeStep = 0;  // the last time the output pin was toggled
unsigned long debounceDelayStep = 50;    // the debounce time; increase if the output flickers
int lastButtonState = LOW;
int buttonState; 
int Steps; 

// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
    // Nothing to do (Stepper Library sets pins as outputs)
}

int inputStep(){
  Serial.println("Input steps:");
  while (Serial.available() == 0) {
  }
  int speed = Serial.parseInt();
  return speed;
}

void singleStepper(int val) {
  myStepper.step(val);
  stepCount++;
  delay(100);
  Serial.println(val);
}

bool us_dist(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;

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

void loop() {

  int sensorReading = analogRead(A0);
  int motorSpeed = map(sensorReading, 0, 1023, -10, 10);
  int dir = 1; // change direction based on mapping
  if (motorSpeed < 0) dir = -1;
  //Serial.println(motorSpeed);
  
  bool dist = us_dist();

  if (abs(motorSpeed) > 0 && !dist) {
    myStepper.setSpeed(abs(motorSpeed));
    
    // step 1/100 of a revolution:
    myStepper.step(dir*(stepsPerRevolution / 10));
    delay(abs(motorSpeed));
  }
  
        // safe mode
  if (dist) {
    myStepper.setSpeed(0);
  }
}
