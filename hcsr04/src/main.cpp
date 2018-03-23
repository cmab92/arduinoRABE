
/*
 * Author: Bonenberger
 * Purpose: trigger, readout and publish multiple HCSR04 USS
 * Update: all in one msg
 */
#include <ros.h>
#include <std_msgs/Float32.h>

// defines pins numbers
#define TRIGGER1 6
#define TRIGGER2 7
#define TRIGGER3 8
#define ECHO1 10
#define ECHO2 11
#define ECHO3 12
#define BAUDRATE 57600
// ros
ros::NodeHandle nh;
std_msgs::Float32 distance_msg1;
std_msgs::Float32 distance_msg2;
std_msgs::Float32 distance_msg3;
ros::Publisher pub_dist2("pub_dist1", &distance_msg1);
ros::Publisher pub_dist1("pub_dist2", &distance_msg2);
ros::Publisher pub_dist3("pub_dist3", &distance_msg3);

void setup() {
  pinMode(TRIGGER1, OUTPUT); // Sets the trigPin as an Output
  pinMode(TRIGGER2, OUTPUT); // Sets the trigPin as an Output
  pinMode(TRIGGER3, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO1, INPUT); // Sets the echoPin as an Input
  pinMode(ECHO2, INPUT); // Sets the echoPin as an Input
  pinMode(ECHO3, INPUT); // Sets the echoPin as an Input
  Serial.begin(BAUDRATE); // Starts the serial communication
  Serial.println("setup");
  digitalWrite(TRIGGER1, LOW);
  digitalWrite(TRIGGER2, LOW);
  digitalWrite(TRIGGER3, LOW);
  nh.initNode();
  nh.advertise(pub_dist1);
  nh.advertise(pub_dist2);
  nh.advertise(pub_dist3);

  Serial.println("setup");
}

void loop() {
  unsigned long startTime = micros();
  unsigned long startTime1 = micros();
  unsigned long startTime2 = micros();
  unsigned long startTime3 = micros();
  unsigned long passedTime;
  unsigned long sensorTime1 = 0;
  unsigned long sensorTime2 = 0;
  unsigned long sensorTime3 = 0;
  double distance1;
  double distance2;
  double distance3;
  bool echoDet1;
  bool echoDet2;
  bool echoDet3;
  unsigned int sensorCnt = 5;

  while(1){
    passedTime = micros() - startTime;
    echoDet1 = digitalRead(ECHO1);
    echoDet2 = digitalRead(ECHO2);
    echoDet3 = digitalRead(ECHO3);

    if ( (passedTime > 10000) || (passedTime < 0) ) {
      startTime = micros();
      digitalWrite(sensorCnt, HIGH);
      delayMicroseconds(10);
      digitalWrite(sensorCnt, LOW);
      if (sensorCnt > 8){
        sensorCnt = 6;
      }
      else {
        sensorCnt += 1;
      }
      //
      if ( (distance1 < 2.0)&&(distance1 > 0) ) {
        distance1 = distance1;
      }
      else {
        distance1 = 4;
      }
      distance_msg1.data = distance1;
      pub_dist1.publish( &distance_msg1 );
      //
      if ( (distance2 < 2.0)&&(distance2 > 0) ) {
        distance2 = distance2;
      }
      else {
        distance2 = 4;
      }
      distance_msg2.data = distance2;
      pub_dist2.publish( &distance_msg2 );
      //
      if ( (distance3 < 2.0)&&(distance3 > 0) ) {
        distance3 = distance3;
      }
      else {
        distance3 = 4;
      }
      distance_msg3.data = distance3;
      pub_dist3.publish( &distance_msg3 );
      nh.spinOnce();
    }
    else {
      if ( digitalRead(ECHO1) == 1 ) {
        sensorTime1 = micros() - startTime1;
      }
      if ( digitalRead(ECHO2) == 1 ) {
        sensorTime2 = micros() - startTime2;
      }
      if ( digitalRead(ECHO3) == 1 ) {
        sensorTime3 = micros() - startTime3;
      }
      if ( (digitalRead(ECHO1) != echoDet1) && (echoDet1 == 1) ) {
        distance1 = (float)sensorTime1 * 0.0001724;
      }
      if ( (digitalRead(ECHO2) != echoDet2) && (echoDet2 == 1) ) {
        distance2 = (float)sensorTime2 * 0.0001724;
      }
      if ( (digitalRead(ECHO3) != echoDet3) && (echoDet3 == 1) ) {
        distance3 = (float)sensorTime3 * 0.0001724;
      }
    }

    if ( (digitalRead(ECHO1) != echoDet1) && (echoDet1 == 0) ) { startTime1 = micros(); }
    if ( (digitalRead(ECHO2) != echoDet2) && (echoDet2 == 0) ) { startTime2 = micros(); }
    if ( (digitalRead(ECHO3) != echoDet3) && (echoDet3 == 0) ) { startTime3 = micros(); }

  }
}
