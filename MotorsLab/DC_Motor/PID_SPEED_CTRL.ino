// LETS CONTROL THE SPEED OF THE MOTOR
#define ENCA 2 //Blue wire; encoder A
#define ENCB 3 // Purple wire; encoder B
#define PWM 5 // Enable pin capable of PWM
#define IN1 8 // Enable Dir1 pin
#define IN2 9 //Enable Dir2 pin

const int potPin = A1; //Potentiometer
volatile int motor_posn = 0;
int motor_speed, mapped_speed;

long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float v3Filt = 0;
float v3Prev = 0;

float v3 = 0;
float eintegral = 0;
float eprev = 0;
void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);

  pinMode(PWM, OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}



void loop() {
  // int pwr = 100/3.0*micros()/1.0e6;
  // int pwr = 255;
  // int dir = 1;
  float target_speed = analogRead(potPin);
  target_speed = map(target_speed,0,1023,0,255);
  // Serial.println(target_speed);

  int pos = 0;
  float velocity2 = 0;

  noInterrupts();
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts();  

  long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  float velocity1 = (pos-posPrev)/deltaT;
  // float velocity1 = fabs((pos - posPrev) * 60 / (43.8 * deltaT));

  
  float velocity3 = (pos-posPrev)*60/(98*deltaT); //This one is most accurate
  posPrev = pos;
  prevT = currT;
  
  //Convert to RPM
  // float v1 = (velocity1/(108))*60.0; //108
  float v1 = (velocity1/(16*43.8))*60.0;
  // float v2 = (velocity2/600)*60.0;
  float v3 = (velocity3/(16*43.8))*60.0;

  v1Filt = 0.854*v1Filt+0.0728*v1+0.0728*v1Prev;
  v1Prev = v1;

  // v2Filt = 0.854*v2Filt+0.0728*v2+0.0728*v2Prev;
  // v2Prev = v2;

  // Serial.print("v1Filt: ");
  // Serial.println(v1Filt);

  v3Filt = 0.854*v3Filt+0.0728*v3+0.0728*v3Prev; //This is the filter velocity
  v3Prev = v3;

  // Serial.print("v3Filt: ");
  // Serial.println(v3Filt);

  float curr_motor_vel;
  curr_motor_vel = v1Filt;
  
  float kp = 1.5; //1.5 
  float ki = 0.025;
  float kd = 0; //0.1
  float target_speed_rpm = target_speed; //(target_speed/100)*60;

  float e = target_speed_rpm - curr_motor_vel;
  Serial.print("Error: ");
  Serial.println(e);
  
  eintegral = eintegral+e*deltaT;

  float dedt = (e-eprev)/(deltaT);
  float u = kp*e+kd*dedt+ki*eintegral;
  // Serial.print(u);
  int dir = -1;
  if (u<0){
    dir = 1;
  }

  int pwr = (int) fabs(u);
  if (pwr>255){
    pwr = 255;
  }

  eprev = 0;

  setMotorSpeed(dir,pwr,PWM, IN1, IN2);

  Serial.print("Target Speed: ");
  Serial.println(target_speed_rpm);
  Serial.print("Current Motor Speed: ");
  Serial.println(curr_motor_vel); 
  delay(500);
  // Serial.println();
}

void setMotorSpeed(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, pwmVal);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  // if (dir == 1){
  //   digitalWrite(in1,HIGH);
  //   digitalWrite(in2,LOW);
  // }
  // else if (dir == -1){
  //   digitalWrite(in1,LOW);
  //   digitalWrite(in2,HIGH);
  // }
  // else{
  //     digitalWrite(in1,LOW);
  //     digitalWrite(in2,LOW);
  // }
}

void readEncoder(){
  int b = digitalRead(ENCB); //Reads encoder b
  int increment = 0;

  if(b>0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  pos_i = pos_i+increment;

  long currT = micros();
  float deltaT = ((float)(currT-prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}



