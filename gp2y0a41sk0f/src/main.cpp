
/*
cb, 21.03.18
*/
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

#define IR0 A0
#define IR1 A1
#define IR2 A2
#define TSAMPLE 16500 //in us (16500us := measuring cycle of 0a41sk )
#define DIST_THRES 1 // measurent larger than DIST_THRES is set to 0

float timeStamp;

ros::NodeHandle nh;
std_msgs::Float32 dist_IR0_msg;
ros::Publisher pub_dist_IR0("dist_IR0_msg", &dist_IR0_msg);
std_msgs::Float32 dist_IR1_msg;
ros::Publisher pub_dist_IR1("dist_IR1_msg", &dist_IR1_msg);
std_msgs::Float32 dist_IR2_msg;
ros::Publisher pub_dist_IR2("dist_IR2_msg", &dist_IR2_msg);


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
  dist_IR0_msg.data = analogRead( IR0 );
  dist_IR1_msg.data = analogRead( IR1 );
  dist_IR2_msg.data = analogRead( IR2 );
  pub_dist_IR0.publish( &dist_IR0_msg );
  pub_dist_IR1.publish( &dist_IR1_msg );
  pub_dist_IR2.publish( &dist_IR2_msg );
  nh.spinOnce();
}

void setup() {
  // Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub_dist_IR0);
  nh.advertise(pub_dist_IR1);
  nh.advertise(pub_dist_IR2);

  Timer1.initialize(TSAMPLE);
  Timer1.attachInterrupt(measure);
}

void loop() {
  while(1){
      // wait for interrupt (respectively next sample)
  }
}
