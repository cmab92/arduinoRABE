/*
Author: cb
Date: March 18
Purpose:
- publish IMU-data from BNO055
- handle hcsr04
Attention:
*/
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>

// #define TRIGGER1 7
// #define TRIGGER2 9
// #define TRIGGER3 11
// #define TRIGGER4 13
// #define ECHO1 6
// #define ECHO2 8
// #define ECHO3 10
// #define ECHO4 12

Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::NodeHandle nh;
geometry_msgs::Vector3 linacc_msg;
geometry_msgs::Vector3 angvec_msg;
geometry_msgs::Quaternion quat_msg;
// std_msgs::Float32MultiArray dist_msg;
ros::Publisher pub_linacc("linacc_msg", &linacc_msg);
ros::Publisher pub_angvec("angvec_msg", &angvec_msg);
ros::Publisher pub_quat("quat_msg", &quat_msg);
// ros::Publisher pub_dist("dist_msg", &dist_msg);

void getCalReg( void ){
  for ( int i = 0x55; i< 0x6b; i++ ){
    // Serial.println(bno.readBNO(0, i ) );
  }
}

void writeBNO( bool page, byte reg, byte value ){
  // write safely
  while ( !bno.writeBNO( 0, reg, value ) ){
    bno.writeBNO( 0, reg, value );
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

void setup() {

    // pinMode(TRIGGER1, OUTPUT);
    // pinMode(TRIGGER2, OUTPUT);
    // pinMode(TRIGGER3, OUTPUT);
    // pinMode(TRIGGER4, OUTPUT);
    // pinMode(ECHO1, INPUT);
    // pinMode(ECHO2, INPUT);
    // pinMode(ECHO3, INPUT);
    // pinMode(ECHO4, INPUT);
    //
    // digitalWrite(TRIGGER1, LOW);
    // digitalWrite(TRIGGER2, LOW);
    // digitalWrite(TRIGGER3, LOW);
    // digitalWrite(TRIGGER4, LOW);

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
    writeBNO(1, INT_EN_ADDR, (byte)(0b00100000));
    writeBNO(1, INT_MSK_ADDR, (byte)(0b00100000));
    writeBNO(1, ACC_INT_SETTINGS_ADDR, (byte)(0b11100000));

    setCalReg();
    writeBNO(0, SYS_TRIGGER_REG, EXTAL); // external crystal use enabled
    writeBNO(0, OPR_MODE_REG, NDOF_OPR);

    nh.initNode();
    nh.advertise(pub_linacc);
    nh.advertise(pub_angvec);
    nh.advertise(pub_quat);
    // nh.advertise(pub_dist);
}

void loop() {
  // unsigned long passedTime;
  // unsigned long startTime = micros();
  // unsigned long measStartTime[4] = {0, 0, 0, 0};
  // unsigned long sensorTime[4] = {0, 0, 0, 0};
  // float distance[4] = {0, 0, 0, 0};
  // bool echoDet[4] = {0, 0, 0, 0};

  while(1){
    // passedTime = micros() - startTime;
    // for (int i = 0; i<4; i++){
    //   echoDet[i] = digitalRead( (6 + i*2) );
    // }
    // // trigger hcsr04 sequentially
    // for (int i = 0; i<4; i++){
    //   digitalWrite( (7 + i*2) , HIGH);
    //   while ( (passedTime < 20000) || (passedTime < 0) ){
    //     if ( (digitalRead( (6 + 2*i) ) != echoDet[i]) && (echoDet[i] == 1) ) {
    //       sensorTime[i] = micros() - measStartTime[i];
    //       distance[i] = sensorTime[i] * 0.0001724;
    //       if (distance[i] > 2.0){
    //         distance[i] = 2.0;
    //       }
    //     }
    //   }
    // }
    // dist_msg.data = distance;
    // pub_dist.publish( &dist_msg );

    imu::Quaternion quat = bno.getQuat();
    quat_msg.w = quat.w();
    quat_msg.x = quat.x();
    quat_msg.y = quat.y();
    quat_msg.z = quat.z();
    pub_quat.publish( &quat_msg );
    nh.spinOnce();
    imu::Vector<3> vec_linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    linacc_msg.x = vec_linacc.x();
    linacc_msg.y = vec_linacc.y();
    linacc_msg.z = vec_linacc.z();
    pub_linacc.publish( &linacc_msg );
    nh.spinOnce();
    imu::Vector<3> vec_acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    angvec_msg.x = vec_acc.x();
    angvec_msg.y = vec_acc.y();
    angvec_msg.z = vec_acc.z();
    pub_angvec.publish( &angvec_msg );
    nh.spinOnce();
  }
}
























































//
