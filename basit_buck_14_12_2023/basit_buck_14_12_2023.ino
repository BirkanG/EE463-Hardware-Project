void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13,HIGH);
  delayMicroseconds(4);
  digitalWrite(13,LOW);
  delayMicroseconds(9);
}
