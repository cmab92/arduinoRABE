
/*
 * Author: Bonenberger
 * Purpose: trigger, readout and publish multiple HCSR04 USS
 * Update: all in one msg
 todo EQUIDISTANT SAMPLING
 */
#include <ros.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

#define TRIGGER0 6
#define TRIGGER1 7
#define TRIGGER2 8
#define ECHO0 10
#define ECHO1 11
#define ECHO2 12
#define TSAMPLE 16500 //in us (16500us := measuring cycle of 0a41sk )
#define BAUDRATE 57600

double distance[3] = {0, 0, 0};

// ros
ros::NodeHandle nh;
std_msgs::Float32 distance_msg0;
std_msgs::Float32 distance_msg1;
std_msgs::Float32 distance_msg2;
ros::Publisher pub_dist0("pub_dist0", &distance_msg0);
ros::Publisher pub_dist1("pub_dist1", &distance_msg1);
ros::Publisher pub_dist2("pub_dist2", &distance_msg2);

void measure( void ) {
  for (int i = 0; i < 3; i++) {
    if ( (distance[i] < 2.0)&&(distance[i] > 0) ) { distance[i] = distance[i]; }
    else { distance[i] = 4; }
  }
  // send data
  distance_msg0.data = distance[0];
  pub_dist0.publish( &distance_msg0 );
  distance_msg1.data = distance[1];
  pub_dist1.publish( &distance_msg1 );
  distance_msg2.data = distance[2];
  pub_dist2.publish( &distance_msg2 );
  nh.spinOnce();
}

void setup() {
  // Serial.begin(BAUDRATE);

  pinMode(TRIGGER0, OUTPUT);
  pinMode(TRIGGER1, OUTPUT);
  pinMode(TRIGGER2, OUTPUT);
  pinMode(ECHO0, INPUT);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  digitalWrite(TRIGGER0, LOW);
  digitalWrite(TRIGGER1, LOW);
  digitalWrite(TRIGGER2, LOW);
  nh.initNode();
  nh.advertise(pub_dist0);
  nh.advertise(pub_dist1);
  nh.advertise(pub_dist2);

  Timer1.initialize(TSAMPLE);
  Timer1.attachInterrupt(measure);
}

void loop() {
  unsigned long startTime = micros();
  unsigned long sensorStartTime[3] = {0, 0, 0};
  unsigned long passedTime;
  unsigned long sensorTime[3] = {0, 0, 0};
  int echoDet[3] = {digitalRead(ECHO0), digitalRead(ECHO1), digitalRead(ECHO2)};
  while(1) {
    passedTime = micros() - startTime;
    echoDet[0] = digitalRead(ECHO0);
    echoDet[1] = digitalRead(ECHO1);
    echoDet[2] = digitalRead(ECHO2);
    if ( (passedTime > 10000) || (passedTime < 0) ) {    // detects targets in ~1.70m distance
      startTime = micros();
      digitalWrite(TRIGGER0, HIGH);
      digitalWrite(TRIGGER1, HIGH);
      digitalWrite(TRIGGER2, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER0, LOW);
      digitalWrite(TRIGGER1, LOW);
      digitalWrite(TRIGGER2, LOW);
    }
    else {
      for (int i = 0; i<3; i++) {
        // update sensorTime
        if ( digitalRead( (10+i) ) == 1 ) {
          sensorTime[i] = micros() - sensorStartTime[i];
        }
        // check for falling edge of echo ... calculate distance in m
        if ( (digitalRead( (10+i) ) != echoDet[i]) && (echoDet[i] == 1) ) {
          distance[i] = (float)sensorTime[i] * 0.0001724;
        }
      }
    }
    // check for rising edge of echo
    for (int i = 0; i<3; i++) {
      if ( (digitalRead( (10+i) ) != echoDet[i]) && (echoDet[i] == 0) ) { sensorStartTime[i] = micros(); }
    }
  }
}
