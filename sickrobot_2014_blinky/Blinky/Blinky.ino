void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(8, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    char a = Serial.read();
    if (a == 'a' || a == 'A') {
      digitalWrite(8, HIGH);
    } else if (a == 'b' || a == 'B') {
      digitalWrite(8, LOW);
    }
  }
}
