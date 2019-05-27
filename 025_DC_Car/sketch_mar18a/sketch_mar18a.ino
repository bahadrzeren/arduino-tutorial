//#include <Wire.h>
///*
//The circuit:
// VCC: 5V
// GND: ground
// SCL: UNO SLC
// SDA: UNO SDA
// 
// This example code is in the public domain.
//*/
//
//// Registers for ADXL345
//#define ADXL345_ADDRESS (0xA6 >> 1)  // address for device is 8 bit but shift to the
//                                     // right by 1 bit to make it 7 bit because the
//                                     // wire library only takes in 7 bit addresses
//#define ADXL345_REGISTER_XLSB (0x32)
//
//int accelerometer_data[3];
//// void because this only tells the cip to send data to its output register
//// writes data to the slave's buffer
//void i2c_write(int address, byte reg, byte data) {
//  // Send output register address
//  Wire.beginTransmission(address);
//  // Connect to device
//  Wire.write(reg);
//  // Send data
//  Wire.write(data); //low byte
//  Wire.endTransmission();
//}
//
//// void because using pointers
//// microcontroller reads data from the sensor's input register
//void i2c_read(int address, byte reg, int count, byte* data) {
//  // Used to read the number of data received
//  int i = 0;
//  // Send input register address
//  Wire.beginTransmission(address);
//  // Connect to device
//  Wire.write(reg);
//  Wire.endTransmission();
//
//  // Connect to device
//  Wire.beginTransmission(address);
//  // Request data from slave
//  // Count stands for number of bytes to request
//  Wire.requestFrom(address, count);
//  while(Wire.available()) // slave may send less than requested
//  {
//    char c = Wire.read(); // receive a byte as character
//    data[i] = c;
//    i++;
//  }
//  Wire.endTransmission();
//}
//
//byte bytes[6];
//
//void init_adxl345() {
//  memset(bytes,0,6);
//
//  byte data = 0;
//
//  i2c_write(ADXL345_ADDRESS, 0x31, 0x0B);   // 13-bit mode  +_ 16g
//  i2c_write(ADXL345_ADDRESS, 0x2D, 0x08);   // Power register
//
//  i2c_write(ADXL345_ADDRESS, 0x1E, 0x00);   // x
//  i2c_write(ADXL345_ADDRESS, 0x1F, 0x00);   // Y
//  i2c_write(ADXL345_ADDRESS, 0x20, 0x05);   // Z
// 
//  // Check to see if it worked!
//  i2c_read(ADXL345_ADDRESS, 0X00, 1, &data);
//  if(data==0xE5)
//    Serial.println("It works!");
//  else
//    Serial.println("Failed!");
//}
//
//void read_adxl345() {
//
//  // Read 6 bytes from the ADXL345
//  i2c_read(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, 6, bytes);
//  // Unpack data
//  for (int i=0;i<3;++i) {
//    accelerometer_data[i] = (int)bytes[2*i] + (((int)bytes[2*i + 1]) << 8);
//  }
//}
//
//int a1 = 6;
//int l1 = 7;
//int r1 = 8;
//int l2 = 9;
//int r2 = 10;
//int a2 = 11;
//
//void setup() {
//  pinMode(a1, OUTPUT);
//  pinMode(l1, OUTPUT);
//  pinMode(r1, OUTPUT);
//  pinMode(a2, OUTPUT);
//  pinMode(l2, OUTPUT);
//  pinMode(r2, OUTPUT);
//
//  Wire.begin();
//  for(int i=0; i<3; ++i) {
//    accelerometer_data[i] = 0;
//  }
//  init_adxl345();
//
////  Serial.begin(9600);
//}
//
//double theta = 0.0;
//double thetaDelta = 0.0;
//double prevTheta = 0.0;
//double prevThetaDelta = 0.0;
//
//int trottle = 0;
//
//void loop() {
//  read_adxl345();
//
//  theta = float(accelerometer_data[2]) * 3.9 / 1000;
//  thetaDelta = theta - prevTheta;
//  trottle = 392 * theta;
//
////  Serial.print(theta);
////  Serial.print(", ");
////  Serial.print(trottle);
////  Serial.print("\n");
//
//  if (theta > 0.0) {
//    analogWrite(a1, trottle);
//    analogWrite(a2, trottle);
//    digitalWrite(l1, HIGH);
//    digitalWrite(r1, LOW);
//    digitalWrite(l2, HIGH);
//    digitalWrite(r2, LOW);
//  } else {
//    analogWrite(a1, -trottle);
//    analogWrite(a2, -trottle);
//    digitalWrite(l1, LOW);
//    digitalWrite(r1, HIGH);
//    digitalWrite(l2, LOW);
//    digitalWrite(r2, HIGH);
//  }
//
//  prevTheta = theta;
//  prevThetaDelta = thetaDelta;
//  delay(10);
//}


#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double setpoint = 176.0;
double input, output;

//adjust these values to fit your own design
double Kp = 21;
double Kd = 0.8;
double Ki = 140;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;

//MOTOR CONTROLLER
//int a1 = 6;
//int l1 = 7;
//int r1 = 8;
//int l2 = 9;
//int r2 = 10;
//int a2 = 11;
int ENA = 6;
int IN1 = 7;
int IN2 = 8;
int IN3 = 9;
int IN4 = 10;
int ENB = 11;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255); 
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else
    if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize)
        fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      input = ypr[1] * 180/M_PI + 180;
    }
}

