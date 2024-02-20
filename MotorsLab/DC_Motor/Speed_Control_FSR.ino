// LETS CONTROL THE SPEED OF THE MOTOR TURNING WITH THE FORCE SENSOR
#define ENCA 2 // Blue wire; encoder A
#define ENCB 3 // Purple wire; encoder B
#define PWM 5  // Enable pin capable of PWM
#define IN1 8  // Enable Dir1 pin
#define IN2 9  // Enable Dir2 pin
#define FORCE_SENSOR_PIN A0

long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // Set the motor speed to 100
  setMotor(1, 100, PWM, IN1, IN2);
}

void loop() {
  int fsrReading = analogRead(FORCE_SENSOR_PIN);
  
  // If force is detected, stop the motor
  if (fsrReading > 10) {
    setMotor(0, 0, PWM, IN1, IN2); // Stop the motor
    Serial.println("Force detected. Motor stopped.");
  }

  else{
    setMotor(1, 100, PWM, IN1, IN2);
    Serial.println("Motor spinning at speed 100");
  }

  int pos = 0;
  float velocity2 = 0;
  noInterrupts();
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); 

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = (velocity1 / 108) * 60.0;
  float v2 = (velocity2 / 210) * 60.0;

  // Set a target Speed
  // Serial.print(v1);
  // Serial.print(" ");
  // Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB); // Reads encoder b
  int increment = (b > 0) ? 1 : -1;
  
  pos_i = pos_i + increment;

  long currT = micros();
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}