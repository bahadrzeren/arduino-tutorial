#include <Wire.h>
/*
The circuit:
 VCC: 5V
 GND: ground
 SCL: UNO SLC
 SDA: UNO SDA
 
 This example code is in the public domain.
*/

// Registers for ADXL345
#define ADXL345_ADDRESS (0xA6 >> 1)  // address for device is 8 bit but shift to the
                                     // right by 1 bit to make it 7 bit because the
                                     // wire library only takes in 7 bit addresses
#define ADXL345_REGISTER_XLSB (0x32)

int accelerometer_data[3];
// void because this only tells the cip to send data to its output register
// writes data to the slave's buffer
void i2c_write(int address, byte reg, byte data) {
  // Send output register address
  Wire.beginTransmission(address);
  // Connect to device
  Wire.write(reg);
  // Send data
  Wire.write(data); //low byte
  Wire.endTransmission();
}

// void because using pointers
// microcontroller reads data from the sensor's input register
void i2c_read(int address, byte reg, int count, byte* data) {
  // Used to read the number of data received
  int i = 0;
  // Send input register address
  Wire.beginTransmission(address);
  // Connect to device
  Wire.write(reg);
  Wire.endTransmission();

  // Connect to device
  Wire.beginTransmission(address);
  // Request data from slave
  // Count stands for number of bytes to request
  Wire.requestFrom(address, count);
  while(Wire.available()) // slave may send less than requested
  {
    char c = Wire.read(); // receive a byte as character
    data[i] = c;
    i++;
  }
  Wire.endTransmission();
}

byte bytes[6];

void init_adxl345() {
  memset(bytes,0,6);

  byte data = 0;

  i2c_write(ADXL345_ADDRESS, 0x31, 0x0B);   // 13-bit mode  +_ 16g
  i2c_write(ADXL345_ADDRESS, 0x2D, 0x08);   // Power register

  i2c_write(ADXL345_ADDRESS, 0x1E, 0x00);   // x
  i2c_write(ADXL345_ADDRESS, 0x1F, 0x00);   // Y
  i2c_write(ADXL345_ADDRESS, 0x20, 0x05);   // Z
 
  // Check to see if it worked!
  i2c_read(ADXL345_ADDRESS, 0X00, 1, &data);
  if(data==0xE5)
    Serial.println("It works!");
  else
    Serial.println("Failed!");
}

void read_adxl345() {

  // Read 6 bytes from the ADXL345
  i2c_read(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, 6, bytes);
  // Unpack data
  for (int i=0;i<3;++i) {
    accelerometer_data[i] = (int)bytes[2*i] + (((int)bytes[2*i + 1]) << 8);
  }
}

int a1 = 6;
int l1 = 7;
int r1 = 8;
int l2 = 9;
int r2 = 10;
int a2 = 11;

void setup() {
  pinMode(a1, OUTPUT);
  pinMode(l1, OUTPUT);
  pinMode(r1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(r2, OUTPUT);

  Wire.begin();
  for(int i=0; i<3; ++i) {
    accelerometer_data[i] = 0;
  }
  init_adxl345();

//  Serial.begin(9600);
}

double theta = 0.0;
double thetaDelta = 0.0;
double prevTheta = 0.0;
double prevThetaDelta = 0.0;

int trottle = 0;

void loop() {
  read_adxl345();

  theta = float(accelerometer_data[2]) * 3.9 / 1000;
  thetaDelta = theta - prevTheta;
  trottle = 392 * theta;

//  Serial.print(theta);
//  Serial.print(", ");
//  Serial.print(trottle);
//  Serial.print("\n");

  if (theta > 0.0) {
    analogWrite(a1, trottle);
    analogWrite(a2, trottle);
    digitalWrite(l1, HIGH);
    digitalWrite(r1, LOW);
    digitalWrite(l2, HIGH);
    digitalWrite(r2, LOW);
  } else {
    analogWrite(a1, -trottle);
    analogWrite(a2, -trottle);
    digitalWrite(l1, LOW);
    digitalWrite(r1, HIGH);
    digitalWrite(l2, LOW);
    digitalWrite(r2, HIGH);
  }

  prevTheta = theta;
  prevThetaDelta = thetaDelta;
  delay(10);
}
