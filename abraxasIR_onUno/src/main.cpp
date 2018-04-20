
/*
cb, 21.03.18
*/
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

#define IR8 A0
#define IR9 A1
#define IR10 A2
#define IR11 A3
#define TSAMPLE 12000 //in us (16500us := measuring cycle of 0a41sk )
#define DIST_THRES 1 // measurent larger than DIST_THRES is set to 0

float timeStamp;

ros::NodeHandle nh;
std_msgs::Float32 dist_IR8_msg;
ros::Publisher pub_dist_IR8("dist_IR8_msg", &dist_IR8_msg);
std_msgs::Float32 dist_IR9_msg;
ros::Publisher pub_dist_IR9("dist_IR9_msg", &dist_IR9_msg);
std_msgs::Float32 dist_IR10_msg;
ros::Publisher pub_dist_IR10("dist_IR10_msg", &dist_IR10_msg);
std_msgs::Float32 dist_IR11_msg;
ros::Publisher pub_dist_IR11("dist_IR11_msg", &dist_IR11_msg);


// float calcDist( float analogVal){ // curve fitting to a*x^b+c
//   float dist;
//   dist = 2.475*pow(analogVal,-0.343)-0.256;
//   if ( (dist>0) && (dist<DIST_THRES) ){
//     dist = dist;
//   } // added after testlauf01
//   else {
//     dist = 2;
//   }
//   return dist;
// }

void measure( void ) {
  analogRead(IR8);
  dist_IR8_msg.data = analogRead( IR8 );
  analogRead(IR9);
  dist_IR9_msg.data = analogRead( IR9 );
  analogRead(IR10);
  dist_IR10_msg.data = analogRead( IR10 );
  analogRead(IR11);
  dist_IR11_msg.data = analogRead( IR11 );
  pub_dist_IR8.publish( &dist_IR8_msg );
  pub_dist_IR9.publish( &dist_IR9_msg );
  pub_dist_IR10.publish( &dist_IR10_msg );
  pub_dist_IR11.publish( &dist_IR11_msg );
  nh.spinOnce();
}

void setup() {
  // Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub_dist_IR8);
  nh.advertise(pub_dist_IR9);
  nh.advertise(pub_dist_IR10);
  nh.advertise(pub_dist_IR11);

  Timer1.initialize(TSAMPLE);
  Timer1.attachInterrupt(measure);
}

void loop() {
  while(1){
      // wait for interrupt (respectively next sample)
  }
}
