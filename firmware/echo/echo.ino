void setup() {
  while (!Serial);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    Serial.print(Serial.read());
  }
}
