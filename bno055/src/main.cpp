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
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

#define ONEQUATERNION 16384 // = 2^14 LSB
#define T_SAMPLE 20000

#define ECHO1 13
#define TRIGGER1 12
#define ECHO2 11
#define TRIGGER2 10
#define ECHO3 9
#define TRIGGER3 8
#define ECHO4 7
#define TRIGGER4 6
#define BNO_IR 8
#define BNO_IR_OUT 7

void measureImu( void );
volatile byte state = LOW;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::NodeHandle nh;
geometry_msgs::Vector3 linacc_msg;
geometry_msgs::Vector3 angvec_msg;
geometry_msgs::Quaternion quat_msg;
std_msgs::Float32 dist_msg;
ros::Publisher pub_linacc("linacc_msg", &linacc_msg);
ros::Publisher pub_angvec("angvec_msg", &angvec_msg);
ros::Publisher pub_quat("quat_msg", &quat_msg);
ros::Publisher pub_dist("dist_msg", &dist_msg);

void getCalReg( void ){
  for ( int i = 0x55; i< 0x6b; i++ ){
    // // Serial.println(bno.readBNO(0, i ) );
  }
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

void waitingState( unsigned long startTime, unsigned long targetTime ){
  while ( (micros() - startTime) < targetTime ) {
    delay(5);
  }
}

void measureImu( void ){
  imu::Quaternion quat = bno.getQuat();
  quat_msg.w = (quat.w()/ONEQUATERNION);
  quat_msg.x = (quat.x()/ONEQUATERNION);
  quat_msg.y = (quat.y()/ONEQUATERNION);
  quat_msg.z = (quat.z()/ONEQUATERNION);
  pub_quat.publish( &quat_msg );
  imu::Vector<3> vec_linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  linacc_msg.x = vec_linacc.x();
  linacc_msg.y = vec_linacc.y();
  linacc_msg.z = vec_linacc.z();
  pub_linacc.publish( &linacc_msg );
  imu::Vector<3> vec_acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  angvec_msg.x = vec_acc.x();
  angvec_msg.y = vec_acc.y();
  angvec_msg.z = vec_acc.z();
  pub_angvec.publish( &angvec_msg );
}

void triggerMeas( void ){
  measureImu();
  nh.spinOnce();
}

void bnoInterrupt ( void ) {
  // ISR ...
  state = !state;
  digitalWrite(BNO_IR_OUT, state);
}

void setup() {

  pinMode(TRIGGER1, OUTPUT);
  pinMode(TRIGGER2, OUTPUT);
  pinMode(TRIGGER3, OUTPUT);
  pinMode(TRIGGER4, OUTPUT);
  pinMode(BNO_IR_OUT, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(ECHO3, INPUT);
  pinMode(ECHO4, INPUT);
  pinMode(BNO_IR, INPUT);

  digitalWrite(TRIGGER1, LOW);
  digitalWrite(TRIGGER2, LOW);
  digitalWrite(TRIGGER3, LOW);
  digitalWrite(TRIGGER4, LOW);
  digitalWrite(BNO_IR_OUT, LOW);

  Serial.begin(57600);
  Wire.begin();
  delay(50);

  if((byte)(bno.readBNO(0, 0x00)) != ID)
  {
    delay(500);
    while((byte)(bno.readBNO(0, 0x00)) != ID){
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

  // set interrupts:
  writeBNO(1, INT_EN_ADDR, 0x20);
  writeBNO(1, INT_MSK_ADDR, 0x20);
  writeBNO(1, ACC_INT_SETTINGS_ADDR, 0xe0);
  writeBNO(1, ACC_HG_DURATION_ADDR, 0x00);
  writeBNO(1, ACC_HG_THRES_ADDR, 0xf0);
  Serial.println(bno.readBNO(1, INT_EN_ADDR),HEX);
  Serial.println(bno.readBNO(1, INT_MSK_ADDR),HEX);
  Serial.println(bno.readBNO(1, ACC_INT_SETTINGS_ADDR),HEX);
  Serial.println(bno.readBNO(1, ACC_HG_DURATION_ADDR),HEX);
  Serial.println(bno.readBNO(1, ACC_HG_THRES_ADDR),HEX);

  setCalReg();
  writeBNO(0, SYS_TRIGGER_REG, EXTAL); // external crystal use enabled
  writeBNO(0, OPR_MODE_REG, NDOF_OPR);
  writeBNO(0, UNIT_SEL_ADDR, 0b10000000); // unit selection

  // nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_linacc);
  nh.advertise(pub_angvec);
  nh.advertise(pub_quat);

  attachInterrupt(digitalPinToInterrupt(BNO_IR), bnoInterrupt, HIGH);
}

void loop() {
  getCalReg();
  while(1){
    triggerMeas();
    writeBNO(0, 0x3f, 0xc0); // reset interrupt pin, set extal
  }
}
