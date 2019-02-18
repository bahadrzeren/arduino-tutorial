int potpin = 0; // initialize analog pin 0
int val = 0;  // define val, assign initial value 0
void setup() {
  Serial.begin(9600);     // set baud rate at 9600
}
void loop() {
  delay(50);                // wait for 0.05 second
  val = analogRead(potpin);   // read the analog value of analog pin 0, and assign it to val 
  Serial.println(val);      // display val’s value
}

