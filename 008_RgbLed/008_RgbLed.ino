int redpin = 11; //select the pin for the red LED
int bluepin =10; // select the pin for the blue LED
int greenpin =9;// select the pin for the green LED
int basepin =9;// select the pin for the base LED

int counter;
int val;

void setup() {
  pinMode(redpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(greenpin, OUTPUT);
}

void loop() 
{
  for (counter = 0; counter < 3; counter++) {
for(val=0; val<255; val++)
  {
   analogWrite(basepin + counter, val);
   analogWrite(basepin + ((counter + 1) % 3), 0);
   analogWrite(basepin + ((counter + 2) % 3), 0);
   delay(5);
  }
for(val=255; val>0; val--)
  {
   analogWrite(basepin + counter, val);
   analogWrite(basepin + ((counter + 1) % 3), 0);
   analogWrite(basepin + ((counter + 2) % 3), 0);
   delay(5);
  }
  }
}
