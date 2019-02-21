#include <Stepper.h>

#define STEPS 100

Stepper stepper(STEPS, 8, 9, 10, 11);

int previous = 0;

void setup() {
  stepper.setSpeed(90);
  Serial.begin(9600);// connect to serial port, set baud rate at “9600”
}

void loop() {
//  int val = analogRead(0);
//  Serial.print(val);
//  Serial.print("; ");
//  Serial.println(val - previous);
//  stepper.step(val - previous);
//  previous = val;
  int val = Serial.read();// read serial port value
  if (val >= '0') {
    val=val-'0';
    Serial.println(val);
    stepper.step(val);
  }
}

