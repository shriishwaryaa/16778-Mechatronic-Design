#define RETRACT_PIN 9
#define EXTEND_PIN 10
#define POSITION_PIN A0
#define POT_PIN A1

int RC_Location;
int RC_Duration;
int Actual_Location;
int DeadBand = 10;

void setup() {
  // put your setup code here, to run once:
  pinMode(RETRACT_PIN, OUTPUT);
  pinMode(EXTEND_PIN, OUTPUT);

  digitalWrite(EXTEND_PIN, LOW);
  digitalWrite(RETRACT_PIN, LOW);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Read the actual location friom the actuator potentiometer
  int RC_Location = analogRead(POT_PIN);

  // Fully retracted is 1016
  // Fully extended is 96

  Actual_Location = analogRead(POSITION_PIN);
  Serial.println(Actual_Location);
  if ((Actual_Location <= RC_Location + DeadBand) && (Actual_Location >= RC_Location - DeadBand)) {
    // If actual location is close to commanded position, do nothing
    Serial.println("NOTHING");
    digitalWrite(EXTEND_PIN, LOW);
    digitalWrite(RETRACT_PIN, LOW);
  }

  if (Actual_Location > RC_Location + DeadBand) {
    Serial.println("EXTEND");
    digitalWrite(EXTEND_PIN, LOW);
    digitalWrite(RETRACT_PIN, HIGH);
  }

  if (Actual_Location < RC_Location - DeadBand) {
    Serial.println("RETRACT");
    digitalWrite(EXTEND_PIN, HIGH);
    digitalWrite(RETRACT_PIN, LOW);
  }

  delay(50);
}
