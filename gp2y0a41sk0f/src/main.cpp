
/*
cb, 21.03.18
*/
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

#define IR4 A0
#define IR5 A1
#define IR6 A2
#define IR7 A3
#define TSAMPLE 12000 //in us (16500us := measuring cycle of 0a41sk )
#define DIST_THRES 1 // measurent larger than DIST_THRES is set to 0

float timeStamp;

ros::NodeHandle nh;
std_msgs::Float32 dist_IR4_msg;
ros::Publisher pub_dist_IR4("dist_IR4_msg", &dist_IR4_msg);
std_msgs::Float32 dist_IR5_msg;
ros::Publisher pub_dist_IR5("dist_IR5_msg", &dist_IR5_msg);
std_msgs::Float32 dist_IR6_msg;
ros::Publisher pub_dist_IR6("dist_IR6_msg", &dist_IR6_msg);
std_msgs::Float32 dist_IR7_msg;
ros::Publisher pub_dist_IR7("dist_IR7_msg", &dist_IR7_msg);


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
  analogRead(IR4);
  dist_IR4_msg.data = analogRead( IR4 );
  analogRead(IR5);
  dist_IR5_msg.data = analogRead( IR5 );
  analogRead(IR6);
  dist_IR6_msg.data = analogRead( IR6 );
  analogRead(IR7);
  dist_IR7_msg.data = analogRead( IR7 );
  pub_dist_IR4.publish( &dist_IR4_msg );
  pub_dist_IR5.publish( &dist_IR5_msg );
  pub_dist_IR6.publish( &dist_IR6_msg );
  pub_dist_IR7.publish( &dist_IR7_msg );
  nh.spinOnce();
}

void setup() {
  // Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub_dist_IR4);
  nh.advertise(pub_dist_IR5);
  nh.advertise(pub_dist_IR6);
  nh.advertise(pub_dist_IR7);

  Timer1.initialize(TSAMPLE);
  Timer1.attachInterrupt(measure);
}

void loop() {
  while(1){
      // wait for interrupt (respectively next sample)
  }
}
