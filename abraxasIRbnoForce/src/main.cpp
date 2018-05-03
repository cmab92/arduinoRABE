/*
Author: cb
Date: April 18
Purpose:
 - read analog inputs (ir-data and force sensors)
 - read bno data
 - write data to serial
 - gray coding analog data (?)
 - write calibration registers of bno
 -
Attention:
 - no high acceleration interrupt of bno (!)
 - check BAUDRATE
 - check TSAMPLE
 - check NOI
 - sda and scl on pin 20 / 21
 - supply voltage for voltage divider of force sensor on pin 2 (R2 = 15)
 - only for atmega2560
 - no interrupt control (conflict with i2c)
 - quaternion data not normalized ( normalize by 16384 = 2¹⁴ LSB)
 -
*/

#include <Wire.h>
#include <Adafruit_BNO055.h>

#define TSAMPLE 16496 //in us (16500us := measuring cycle of 0a41sk/0a51sk/0a21ysk/0a02yk )
#define NOI 14  // number of analog inputs (12 IR + 2 iefsr-force-sensors)
#define VCCVD 2 // Vcc for voltage divider (force sensor)
#define ONEQUATERNION 16384 // = 2^14 LSB
#define BAUDRATE 57600
#define BNOPS0 2 // interrupt pin
#define BNOPS1 3 // address pin
#define BNOINT 52 // interrupt pin
#define BNOADR 50 // address pin

void writeBNO( void );
void setCalReg( void );

int dataSelBno = 0;  // quat, linacc or angvec data

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned int binaryToGray(unsigned short num) {
  return (num>>1) ^ num;
}

void writeBNO( bool page, byte reg, byte value ){
  // write safely
  while ( !bno.writeBNO( page, reg, value ) ){
    bno.writeBNO( page, reg, value );
  }
}

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

void measure( void ) {
  for (int i = 0; i<NOI; i++){
    analogRead(i); // dummy measurement for adc
    delayMicroseconds(50); // "cooldown" adc
    // Serial.print( binaryToGray(analogRead(i)) );
    Serial.print( analogRead(i) );
    Serial.print( "," );
  }
  if ( dataSelBno!=1 && dataSelBno!=2 ) {
    imu::Quaternion quat = bno.getQuat();
    Serial.print( quat.w() );
    Serial.print( "," );
    Serial.print( quat.x() );
    Serial.print( "," );
    Serial.print( quat.y() );
    Serial.print( "," );
    Serial.print( quat.z() );
    Serial.print( "," );
    Serial.print( dataSelBno );
    Serial.print( "," );
    dataSelBno = 1;
  }
  else if ( dataSelBno==1 ) {
    imu::Vector<3> vec_linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    Serial.print( vec_linacc.x(), 5 );
    Serial.print( "," );
    Serial.print( vec_linacc.y(), 5 );
    Serial.print( "," );
    Serial.print( vec_linacc.z(), 5 );
    Serial.print( "," );
    Serial.print( 0.0 ); // dummy, for consistent format
    Serial.print( "," );
    Serial.print( dataSelBno );
    Serial.print( "," );
    dataSelBno = 2;
  }
  else if ( dataSelBno==2 ) {
    imu::Vector<3> vec_acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    Serial.print( vec_acc.x(), 5 );
    Serial.print( "," );
    Serial.print( vec_acc.y(), 5 );
    Serial.print( "," );
    Serial.print( vec_acc.z(), 5 );
    Serial.print( "," );
    Serial.print( 0.0 ); // dummy, for consistent format
    Serial.print( "," );
    Serial.print( dataSelBno );
    Serial.print( "," );
    dataSelBno = 0;
  }
  Serial.print("\n");
}

void setup(){
  pinMode(VCCVD, OUTPUT);
  pinMode(BNOADR, OUTPUT);
  pinMode(BNOINT, INPUT);
  pinMode(BNOPS0, OUTPUT);
  pinMode(BNOPS1, OUTPUT);
  digitalWrite(BNOADR, LOW);

  Serial.begin(BAUDRATE);
  delay(50);
  Wire.begin();
  delay(50);

  if((byte)(bno.readBNO(0, 0x00)) != ID){
    delay(1000);
    while((byte)(bno.readBNO(0, 0x00)) != ID){
      Serial.println("no bno detected");
      break;
    }
  }

  writeBNO(0, OPR_MODE_REG, CONFIG_OPR );

  writeBNO(0, SYS_TRIGGER_REG, RESET); // reset bno
  delay(100);
  while( ID != bno.readBNO(0, STATUS_REG) ){
    delay(25);
  }
  delay(200);

  writeBNO(0, PWR_MODE_REG, NORMAL_PWR); // normal power mode enabled
  delay(10);

  setCalReg();
  writeBNO(0, SYS_TRIGGER_REG, EXTAL); // external crystal use enabled
  writeBNO(0, OPR_MODE_REG, NDOF_OPR);
  writeBNO(0, UNIT_SEL_ADDR, 0b10000000); // unit selection

  delay(100);
}

void loop() {

  unsigned long lastMeasurement = micros();
  measure();

  while (1) {
    if ((micros()-lastMeasurement) >= TSAMPLE){
      lastMeasurement = micros();
      measure();
      Serial.println(micros()-lastMeasurement);
    }
  }
}
