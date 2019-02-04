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

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

void setCalReg( void ){
  // write calibration offset
  /*
    the cal-values below are the mean of following 10 cal.-results:
    vals = [
          0   244   229    13   248   255    17   236   224   255
          0   255   255     0   255   255     0   255   255   255
         11    16    24     4    16    18    31   248   255    13
          0     0     0     0     0     0     0   255   255     0
          9    14    14    12    20    15     7     9     6     4
          0     0     0     0     0     0     0     0     0     0
         88    92    84   111    85   110    55    92    73    66
          0     0     0     0     0     0     0     0     0     0
        147   139   146   130   154   141   141   139   144   142
          0     0     0     0     0     0     0     0     0     0
        122    78    84    89   106    63    74    83    88    80
          0     0     0     0     0     0     0     0     0     0
        249   255   255   255   255   255   255   255   254   254
        255   255   255   255   255   255   255   255   255   255
        252     0     0     0   255     0     0     0   255   255
        255     0     0     0   255     0     0     0   255   255
          5     0     0     0     1     0     0     0     0     0
          0     0     0     0     0     0     0     0     0     0
        232   232   232   232   232   232   232   232   232   232
          3     3     3     3     3     3     3     3     3     3
         37   209    23   247    16    16   125   176   108   214
          3     2     3     2     3     3     3     2     3     2]
  */ // <- thats how the values are chosen, order is flipped from version 1... take care !!!
  byte calvals[22] = {172,179,64,51,11,0,86,0,142,0,87,0,254,255,102,102,1,0,232,3,117,3};
  for ( int i = 0x55; i< 0x6b; i++ ){
    bno.writeBNO(0, i, (byte)(calvals[i-0x55]) );
  }
}

void writeBNO( bool page, byte reg, byte value ){
  bno.writeBNO( page, reg, value );
  /*
  while ( !bno.writeBNO( page, reg, value ) ){
    delay(15);
  }*/
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
  Wire.beginTransmission(ADDRESS);
  writeBNO(0, SYS_TRIGGER_REG, RESET); // reset bno
  delay(50);
  writeBNO(0, SYS_TRIGGER_REG, RESET); // reset bno
  Serial.println(bno.readBNO(0, 0x00));

  if((byte)(bno.readBNO(0, 0x00)) != (byte)(ID)){
    while((byte)(bno.readBNO(0, 0x00)) != (byte)(ID)){
      delay(500);
      Serial.println(bno.readBNO(0, 0x00), HEX);
      Serial.println("no bno detected");
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
  writeBNO(0, UNIT_SEL_ADDR, 0b10000010); // unit selection   linacc = [m/s^2] and angvec = [Rps]
  writeBNO(0, OPR_MODE_REG, NDOF_OPR); // IMU_MODE
  setCalReg();
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
