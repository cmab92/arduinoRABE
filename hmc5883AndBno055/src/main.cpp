/*
Author: cb
Date: March 18
Purpose:
- publish IMU-data from BNO055 ... only handling bno!!!
Attention:
*/
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>


#define ONEQUATERNION 16384 // = 2^14 LSB

#define BNO_IR 2
#define BNO_IR_OUT 3

void measureImu( void );
bool state = LOW;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


// void getCalReg( void ){
//   for ( int i = 0x55; i< 0x6b; i++ ){
//     Serial.println(bno.readBNO(0, i ) );
//   }
// }

void writeBNO( bool page, byte reg, byte value ){
  // make sure to write
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

void bnoInterrupt ( void ) {
  // ISR ...
  state = !state;
  // Serial.println("switch");
  digitalWrite(BNO_IR_OUT, state);
}

void setup() {

  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("no hmc detected");
    while(1);
  }

  // pinMode(BNO_IR_OUT, OUTPUT);
  // pinMode(BNO_IR, INPUT);
  //
  // digitalWrite(BNO_IR_OUT, LOW);

  Serial.begin(57600);
  // Serial.println("setup");
  Wire.begin();
  delay(50);

  if((byte)(bno.readBNO(0, 0x00)) != ID)
  {
    delay(500);
    while((byte)(bno.readBNO(0, 0x00)) != ID){
      Serial.println("no bno detected");
      break;
    }
  }

  writeBNO(0, OPR_MODE_REG, CONFIG_OPR );

  writeBNO(0, SYS_TRIGGER_REG, RESET); // reset bno
  delay(100);
  while( ID != bno.readBNO(0, STATUS_REG) ){
    delay(20);
  }
  delay(100);

  writeBNO(0, PWR_MODE_REG, NORMAL_PWR); // normal power mode enabled
  delay(10);

  // // set interrupts:
  // writeBNO(1, INT_EN_ADDR, 0x20);
  // writeBNO(1, INT_MSK_ADDR, 0x20);
  // writeBNO(1, ACC_INT_SETTINGS_ADDR, 0xe0);
  // writeBNO(1, ACC_HG_DURATION_ADDR, 0x00);
  // writeBNO(1, ACC_HG_THRES_ADDR, 0x90);
  // // Serial.println(bno.readBNO(1, INT_EN_ADDR),HEX);
  // // Serial.println(bno.readBNO(1, INT_MSK_ADDR),HEX);
  // // Serial.println(bno.readBNO(1, ACC_INT_SETTINGS_ADDR),HEX);
  // // Serial.println(bno.readBNO(1, ACC_HG_DURATION_ADDR),HEX);
  // // Serial.println(bno.readBNO(1, ACC_HG_THRES_ADDR),HEX);
  // writeBNO(0, 0x3f, 0xc0); // reset interrupt pin, set extal

  setCalReg();
  writeBNO(0, SYS_TRIGGER_REG, EXTAL); // external crystal use enabled
  writeBNO(0, OPR_MODE_REG, NDOF_OPR);
  writeBNO(0, UNIT_SEL_ADDR, 0b10000000); // unit selection

  // nh.getHardware()->setBaud(115200);

  attachInterrupt(digitalPinToInterrupt(BNO_IR), bnoInterrupt, RISING);
}

void loop() {
  while(1){
    sensors_event_t event;
    mag.getEvent(&event);
    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(event.magnetic.y, event.magnetic.x);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.22;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI;

    Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
    imu::Quaternion quat = bno.getQuat();
    Serial.println(quat.w()/ONEQUATERNION);
    Serial.println(quat.x()/ONEQUATERNION);
    Serial.println(quat.y()/ONEQUATERNION);
    Serial.println(quat.z()/ONEQUATERNION);
    delay(1500); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Serial.println(bno.readBNO(1, INT_EN_ADDR),HEX);
    // Serial.println(bno.readBNO(1, INT_MSK_ADDR),HEX);
    // Serial.println(bno.readBNO(1, ACC_INT_SETTINGS_ADDR),HEX);
    // Serial.println(bno.readBNO(1, ACC_HG_DURATION_ADDR),HEX);
    // Serial.println(bno.readBNO(1, ACC_HG_THRES_ADDR),HEX);
    // writeBNO(0, 0x3f, 0xc0); // reset interrupt pin, set extal
  }
}
