// #define ENCA 2 //Blue wire; encoder A
// #define ENCB 3 // Purple wire; encoder B
// #define PWM 5 // Enable pin capable of PWM
// #define IN1 8 // Enable Dir1 pin
// #define IN2 9 //Enable Dir2 pin

// // const int ENCODER_RESOLUTION_GEARBOX = 700;  // CPR (counts per revolution) for gearbox shaft
// // const int ENCODER_RESOLUTION_MOTOR = 16 * 43.8;  // Adjusted CPR for motor shaft

// const int potPin = A1; //Potentiometer
// volatile int32_t motor_posn = 0;

// int motor_speed, mapped_speed;
// float target = 0;

// long prevT = 0;
// float eprev = 0;
// float eintegral = 0;

// void setup() {
//   Serial.begin(9600);
//   pinMode(ENCA,INPUT_PULLUP);
//   pinMode(ENCB,INPUT_PULLUP);

//   pinMode(PWM, OUTPUT);
//   pinMode(IN1,OUTPUT);
//   pinMode(IN2,OUTPUT);

//   attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
// }


// void loop() {
//   // set target position

//   // target = target*sin(prevT/1e6);
//   target = analogRead(potPin);
//   target = map(target, 0, 1023, 0, 360);
//   // Tunable gains
//   float kp = 1.1;
//   float kd = 0.01;
//   float ki = 0.5;

//   //Time difference
//   long currT = micros(); //Current time in microseconds

//   float deltaT = ((float)(currT-prevT))/1.0e6;
  
//   prevT = currT;
  
//   int pos;
//   noInterrupts();
//   pos = motor_posn;
//   interrupts();
//   // Serial.println("Motor posn set initially");
  
//   //error
//   int e = -target+pos; //Switched this motor will continuously spin if not 

//   //derivative
//   float dedt = (e-eprev)/(deltaT);
//   // Serial.print("Derivative term: ");
//   // Serial.println(dedt);

//   //integral
//   eintegral = eintegral + e*deltaT;

//   //control signal
//   float u = kp*e + kd*dedt + ki*eintegral;

//   // motor power
//   float pwr = fabs(u);
//   if (pwr>255){
//     pwr = 255;
//   }

//   //motor direction
//   int dir = 1;
//   if(u<0){
//     dir = -1;
//   }

//   //signal the motor
//   setMotorPosn(dir,pwr,PWM, IN1, IN2);

//   // store previous error
//   eprev = 0;
//   // eprev = e;

//  Serial.print("Target Posn: ");
//   Serial.print(target);
//   Serial.print(" ");

//   Serial.print("Motor Posn: ");
//   Serial.println(motor_posn);
//   // delay(200000);
//   // Serial.println();
// }

// void setMotorPosn(int dir, int pwmVal, int pwm, int in1, int in2){
//   analogWrite(pwm, pwmVal);
//   if (dir == 1){
//     digitalWrite(in1,HIGH);
//     digitalWrite(in2,LOW);
//   }
//   else if (dir == -1){
//     digitalWrite(in1,LOW);
//     digitalWrite(in2,HIGH);
//   }
//   else{
//       digitalWrite(in1,LOW);
//       digitalWrite(in2,LOW);
//   }
// }

// void readEncoder(){
//   int b = digitalRead(ENCB); //Reads encoder b
//   if(b>0){
//     motor_posn++;
//   }
//   else{
//     motor_posn--;
//   }
// }

#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 //Blue wire; encoder A
#define ENCB 3 // Purple wire; encoder B
#define PWM 5 // Enable pin capable of PWM
#define IN1 8 // Enable Dir1 pin
#define IN2 9 //Enable Dir2 pin
const int potPin = A1; //Potentiometer
volatile int posi = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target = analogRead(potPin);
  target = map(target, 0, 1023, 0, 360);

  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int pos;
  noInterrupts();
  pos = posi;
  interrupts();
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = 0;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}