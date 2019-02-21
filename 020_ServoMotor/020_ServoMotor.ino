#include <Servo.h>

/*
int servopin = 9;// select digital pin 9 for servomotor signal line
int pulsewidth;// initialize width variable
int val;

void servopulse(int servopin, int myangle) {
  pulsewidth = (myangle * 11) + 500;// convert angle to 500-2480 pulse width
  digitalWrite(servopin,HIGH);// set the level of servo pin as “high”
  delayMicroseconds(pulsewidth);// delay microsecond of pulse width
  digitalWrite(servopin,LOW);// set the level of servo pin as “low”
  delay(20-pulsewidth/1000);
}

void setup() {
  pinMode(servopin, OUTPUT);// set servo pin as “output”
  Serial.begin(9600);// connect to serial port, set baud rate at “9600”
  Serial.println("servo=o_seral_simple ready" ) ;
}
void loop() {
  val=Serial.read();// read serial port value
  if(val>='0'&&val<='9') {
    val = val - '0';// convert characteristic quantity to numerical variable
    val = val * (180 / 9);// convert number to angle
    Serial.print("moving servo to ");
    Serial.print(val,DEC);
    Serial.println();
    for(int i=0;i<=50;i++) {
      servopulse(servopin,val);// use the pulse function
    }
  }
}
*/
int val;
Servo myservo;  // define servo variable name
void setup() {
  Serial.begin(9600);// connect to serial port, set baud rate at “9600”
  myservo.attach(9);  // select servo pin(9 or 10)
}
void loop() {
  val = Serial.read();  // read serial port value
  if(val >= '0') {  // && val <= '9') {
    Serial.println(val);
    val = val - '0';// convert characteristic quantity to numerical variable
    Serial.println(val);
    val = val * (180 / 9);// convert number to angle
    Serial.println(val);
    myservo.write(val);  // set rotate angle of the motor
  }
}

