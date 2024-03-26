#define DIRECTION_PIN 2
#define STEPPIN 3

#define STEPS_PER_REV 6400

void setup() {
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEPPIN, OUTPUT);
}

void loop() {
  for (int i = 0; i < STEPS_PER_REV; i++) {
    digitalWrite(DIRECTION_PIN, HIGH);
    digitalWrite(STEPPIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(STEPPIN, LOW);
  }
}