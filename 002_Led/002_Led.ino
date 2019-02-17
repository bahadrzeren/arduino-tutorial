  int ledpin=10;  // define digital interface 10
  void setup() {
    pinMode(ledpin,OUTPUT); // initialize digital pin 10 as output. When using I/O ports on an Arduino, this kind of set up is always needed.
  }
  void loop() {
    for (int i = 0; i < 2; i++) {
      digitalWrite(ledpin,HIGH);  // set the LED on digital pin 10 on. 
      delay(100);
      digitalWrite(ledpin,LOW);   // set the LED on digital pin 10 off.
      delay(100);
    }
    delay(1000);
  }
