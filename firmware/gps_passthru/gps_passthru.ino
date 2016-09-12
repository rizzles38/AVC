void setup() {
  Serial1.begin(9600);
  while (!Serial);
  Serial.begin(9600);
}

void loop() {
  while (Serial1.available() > 0) {
    Serial.write(Serial1.read());
  }
}
