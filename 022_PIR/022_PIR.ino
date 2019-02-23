
byte sensorPin = 3;

void setup() {
  pinMode(sensorPin,INPUT);
  Serial.begin(9600);
}

void loop() {
  byte state = digitalRead(sensorPin);
  if (state == 1)
    Serial.println("Somebody is in this area!");
  else
    if (state == 0)
      Serial.println("No one!");
  delay(500);
}
