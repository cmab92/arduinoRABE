
/*
cb, 19.04.18
*/
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

#define IR0 A0
#define IR1 A1
#define IR2 A2
#define IR3 A3
#define IR4 A4
#define IR5 A5
#define IR6 A6
#define IR7 A7
#define TSAMPLE 1 //in us (16500us := measuring cycle of 0a41sk )
#define DIST_THRES 1 // measurent larger than DIST_THRES is set to 0

float timeStamp;

ros::NodeHandle nh;
std_msgs::Float32 dist_IR0_msg;
ros::Publisher pub_dist_IR0("dist_IR0_msg", &dist_IR0_msg);
std_msgs::Float32 dist_IR1_msg;
ros::Publisher pub_dist_IR1("dist_IR1_msg", &dist_IR1_msg);
std_msgs::Float32 dist_IR2_msg;
ros::Publisher pub_dist_IR2("dist_IR2_msg", &dist_IR2_msg);
std_msgs::Float32 dist_IR3_msg;
ros::Publisher pub_dist_IR3("dist_IR3_msg", &dist_IR3_msg);
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
  analogRead(IR0);
  dist_IR0_msg.data = analogRead( IR0 );
  analogRead(IR1);
  dist_IR1_msg.data = analogRead( IR1 );
  analogRead(IR2);
  dist_IR2_msg.data = analogRead( IR2 );
  analogRead(IR3);
  dist_IR3_msg.data = analogRead( IR3 );
  analogRead(IR4);
  dist_IR4_msg.data = analogRead( IR4 );
  analogRead(IR5);
  dist_IR5_msg.data = analogRead( IR5 );
  analogRead(IR6);
  dist_IR6_msg.data = analogRead( IR6 );
  analogRead(IR7);
  dist_IR7_msg.data = analogRead( IR7 );
  pub_dist_IR0.publish( &dist_IR0_msg );
  pub_dist_IR1.publish( &dist_IR1_msg );
  pub_dist_IR2.publish( &dist_IR2_msg );
  pub_dist_IR3.publish( &dist_IR3_msg );
  pub_dist_IR4.publish( &dist_IR4_msg );
  pub_dist_IR5.publish( &dist_IR5_msg );
  pub_dist_IR6.publish( &dist_IR6_msg );
  pub_dist_IR7.publish( &dist_IR7_msg );
  nh.spinOnce();
}

void setup() {
  // Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub_dist_IR0);
  nh.advertise(pub_dist_IR1);
  nh.advertise(pub_dist_IR2);
  nh.advertise(pub_dist_IR3);
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
