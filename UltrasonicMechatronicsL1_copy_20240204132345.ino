//adapted: https://projecthub.arduino.cc/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-7cabe1

const int trigPin = 9;
const int echoPin = 10;
const int led = 8;

float duration, distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  

  if (distance < 2 || distance > 400)
  {
    Serial.println("Out of Range");
  }
  else
  {
    Serial.print("Distance: ");
    Serial.println(distance);

    if(distance < 20)
    {
      digitalWrite(led, HIGH);   
    }
    else
    {
      digitalWrite(led, LOW);
    }
  }
  delay(100);
}