void setup() {
  pinMode(7, OUTPUT);

}

void loop() {
  digitalWrite(7, LOW);
  delayMicroseconds(8);
  digitalWrite(7,HIGH);
  delayMicroseconds(1);
}
