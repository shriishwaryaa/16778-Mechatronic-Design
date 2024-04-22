void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

String read_string() {
  while (Serial.available() == 0);
  String data = Serial.readStringUntil('\n');
  Serial.println("Read data from serial :");
  Serial.println(data);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  String motor_cmd = "0$5$1$10$2$15$3$20";

  String command = "0$5$1$10$2$15$3$20";
  unsigned int dollar_count = 0;

  while (dollar_count < 7) {
    unsigned int dollar_idx = command.indexOf('$');
    Serial.print("Index of dollar");
    Serial.println(dollar_idx);
    dollar_count++;

    // Fetch 
    String motor_id_string = command.substring(0, dollar_idx);
    Serial.print("Motor ID : ");
    Serial.println(motor_id_string);

    command=command.substring(dollar_idx+1);
    Serial.print("New command");
    Serial.print(command);

    dollar_idx = command.indexOf('$');
    Serial.print("Index of second dollar");
    Serial.println(dollar_idx);
    dollar_count++;

    String angle_data = command.substring(0, dollar_idx);
    Serial.print("Angle info : ");
    Serial.println(angle_data);
    command=command.substring(dollar_idx+1);
    Serial.print("New command");
    Serial.print(command);
  }
  // Spin here to see output
  while(1);
  Serial.println("Done");
}
