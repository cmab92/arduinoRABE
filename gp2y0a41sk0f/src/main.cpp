
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

#define IR0 A0
#define IR1 A1
#define IR2 A2
#define DELAY_MS 2

ros::NodeHandle nh;
std_msgs::Float32 dist_IR0_msg;
ros::Publisher pub_dist_IR0("dist_IR0_msg", &dist_IR0_msg);
std_msgs::Float32 dist_IR1_msg;
ros::Publisher pub_dist_IR1("dist_IR1_msg", &dist_IR1_msg);
std_msgs::Float32 dist_IR2_msg;
ros::Publisher pub_dist_IR2("dist_IR2_msg", &dist_IR2_msg);


float calcDist( float analogVal){ // curve fitting to a*x^b+c
  float dist;
  dist = 2.475*pow(analogVal,-0.343)-0.256;
  return dist;
}

void setup() {
  // Serial.begin(57600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  nh.initNode();
  nh.advertise(pub_dist_IR0);
  nh.advertise(pub_dist_IR1);
  nh.advertise(pub_dist_IR2);
}

void loop() {
  // float analogVal;
  // analogVal = analogRead(IR1);
  // dist_mean += calcDist(analogVal); // first of all no dist_calc on controller ...
  // Serial.println(analogVal);
  delay(DELAY_MS);
  dist_IR0_msg.data = calcDist( analogRead( IR0 ) );
  pub_dist_IR0.publish( &dist_IR0_msg );
  delay(DELAY_MS);
  dist_IR1_msg.data = calcDist( analogRead( IR1 ));
  pub_dist_IR1.publish( &dist_IR1_msg );
  delay(DELAY_MS);
  dist_IR2_msg.data = calcDist( analogRead( IR2 ));
  pub_dist_IR2.publish( &dist_IR2_msg );
  nh.spinOnce();
}
