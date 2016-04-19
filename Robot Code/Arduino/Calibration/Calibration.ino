void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print("Back:  ");
  Serial.println(analogRead(A0));
  Serial.print("Right: ");
  Serial.println(analogRead(A1));
  Serial.print("Front: ");
  Serial.println(analogRead(A2));
  Serial.print("Left:  ");
  Serial.println(analogRead(A3));
  Serial.println("---------");
  delay(1000);
}
