/*
Author: cb
Date: April 18
Purpose:
 - read bno data
 - write data to serial
 - write calibration registers of bno
 -
Attention:
 - no high acceleration interrupt of bno (!)
 - check BAUDRATE
 - check TSAMPLE
 - sda and scl on pin 4 / 5
 - no interrupt control (conflict with i2c)
 - quaternion data not normalized ( normalize by 16384 = 2¹⁴ LSB)
 -
*/

#include <Wire.h>
#include <Adafruit_BNO055.h>

#define TSAMPLE 16496 //in us (16500us := measuring cycle of 0a41sk/0a51sk/0a21ysk/0a02yk )
#define ONEQUATERNION 16384 // = 2^14 LSB
#define BAUDRATE 57600

void writeBNO( void );
void setCalReg( void );

int dataSelBno = 0;  // quat, linacc or angvec data

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned int binaryToGray(unsigned short num) {
  return (num>>1) ^ num;
}

void writeBNO( bool page, byte reg, byte value ){
  while ( !bno.writeBNO( page, reg, value ) ){
    delay(15);
  }
}

void measure( void ) {
  imu::Quaternion d0 = bno.getQuat();
  Serial.print( d0.w(), 0 );
  Serial.print( "," );
  Serial.print( d0.x(), 0 );
  Serial.print( "," );
  Serial.print( d0.y(), 0 );
  Serial.print( "," );
  Serial.print( d0.z(), 0 );
  Serial.print( "," );
  imu::Vector<3> d1 = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  Serial.print( d1.x(), 2 );
  Serial.print( "," );
  Serial.print( d1.y(), 2 );
  Serial.print( "," );
  Serial.print( d1.z(), 2 );
  Serial.print( "," );
  imu::Vector<3> d2 = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print( d2.x(), 2 );
  Serial.print( "," );
  Serial.print( d2.y(), 2 );
  Serial.print( "," );
  Serial.print( d2.z(), 2 );
  Serial.print( "," );
  imu::Vector<3> d3 = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print( d3.x(), 2 );
  Serial.print( "," );
  Serial.print( d3.y(), 2 );
  Serial.print( "," );
  Serial.print( d3.z(), 2 );
  Serial.print( "," );
  Serial.print("\n");
}

void setup(){

  Serial.begin(BAUDRATE);
  delay(50);
  Wire.begin();
  delay(50);

  if((byte)(bno.readBNO(0, 0x00)) != ID){
    delay(500);
    while((byte)(bno.readBNO(0, 0x00)) != ID){
      Serial.println("no bno detected");
      delay(100);
    }
  }

  writeBNO(0, SYS_TRIGGER_REG, RESET); // reset bno
  delay(100);
  while( ID != bno.readBNO(0, STATUS_REG) ){
    delay(25);
  }
  delay(200);

  writeBNO(0, OPR_MODE_REG, CONFIG_OPR );
  writeBNO(0, SYS_TRIGGER_REG, EXTAL); // external crystal use enabled
  writeBNO(0, PWR_MODE_REG, NORMAL_PWR); // normal power mode enabled
  writeBNO(0, UNIT_SEL_ADDR, 0b10000010); // unit selection   linacc = [m/s^2] and angvec = [Dps]
  writeBNO(0, OPR_MODE_REG, NDOF_OPR); // IMU_MODE
  delay(500);
}

void loop() {

  unsigned long lastMeasurement = micros();
  measure();

  while (1) {
    if ((micros()-lastMeasurement) >= TSAMPLE){
      lastMeasurement = micros();
      measure();
      // Serial.println(micros()-lastMeasurement);
    }
  }
}
