  int td = 4;
  int rd1 = 3;
  int rd2 = 4;
  int val1 = 0;
  int val2 = 0;
  void setup() {
//    pinMode(td, OUTPUT);
//    pinMode(rd1, INPUT);
//    pinMode(rd2, INPUT);
    Serial.begin(9600);
  }
  void loop() {
//    analogWrite(td, 150);
//    delay(100);
    val1 = analogRead(rd1);
    val2 = analogRead(rd2);
    Serial.print(val1);
    Serial.print("-");
    Serial.println(val2);
//    delay(100);
//    analogWrite(td,0);
    delay(2000);
  }
