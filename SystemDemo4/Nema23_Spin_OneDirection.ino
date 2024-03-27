#define DIRECTION_PIN 2
#define STEPPIN 3

#define STEPS_PER_REV 3200

void setup() {
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEPPIN, OUTPUT);
}

bool done = false;
int count = 5;

void loop() {
  digitalWrite(DIRECTION_PIN, HIGH);

  for (int i = 0; i < STEPS_PER_REV; i++) {
    digitalWrite(STEPPIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(STEPPIN, LOW);
  }

    // delay(500);
}