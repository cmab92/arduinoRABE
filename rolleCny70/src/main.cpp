/*
wheel circumference left: 0,602 m
wheel circumference right: 0,601 m
*/


#include <Arduino.h>

#define IRPIN_1 2
#define IRPIN_2 3
#define BAUDRATE 57600
#define TSAMPLE 16496   // same as for ir signals
#define TSAMPLE_S 0.016496   // same as for ir signals (in s)
#define TIMEOUT_C 10   // maximum number of zero-tick-periods, until v= 0 m/s

unsigned long timer_1;
unsigned long timer_2;

bool flag_1 = 0;
bool flag_2 = 0;

void count_isr_1() {
  timer_1 = micros();
  flag_1 = 1;
}

void count_isr_2() {
  timer_2 = micros();
  flag_2 = 1;
}

void setup() {
  Serial.begin(BAUDRATE);
  pinMode(IRPIN_1, INPUT);
  pinMode(IRPIN_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRPIN_1), count_isr_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IRPIN_2), count_isr_2, CHANGE);
  timer_1 = micros();
  timer_2 = micros();
}

void loop() {

  static unsigned long samplingCount = micros();
  static const float distPerCount_1 =  1000000*0.6015/72; // ...
  // |-> wheel circumference 0.6015(1)m, 72 steps/blocks, v = m/us = 1000000*m/s
  static const float distPerCount_2 =  1000000*0.6015/72; // ...
  // |-> wheel circumference 0.6015(2)m, 72 steps/blocks, v = m/us
  static int count_1 = 0;
  static int count_2 = 0;
  static unsigned long timer_1_0 = micros();
  static unsigned long timer_2_0 = micros();
  static float velo_1 = 0;
  static float velo_2 = 0;
  static int velo_timeout_1 = 0;
  static int velo_timeout_2 = 0;

  if ((micros()-samplingCount) >= TSAMPLE){
    samplingCount = micros();
    if (count_1==0){
      velo_timeout_1 += 1;
      if (velo_timeout_1>TIMEOUT_C){
        velo_1 = 0;
      }
    }
    else {
      velo_timeout_1 = 0;
      velo_1 = velo_1/count_1;
    }
    if (count_2==0){
      velo_timeout_2 += 1;
      if (velo_timeout_2>TIMEOUT_C){
        velo_2 = 0;
      }
    }
    else {
      velo_timeout_2 = 0;
      velo_2 = velo_2/count_2;
    }
    Serial.print(count_1);
    Serial.print(", ");
    Serial.print(count_2);
    Serial.print(", ");
    Serial.print(velo_1, 4);
    Serial.print(", ");
    Serial.print(velo_2, 4);
    Serial.print("\n");
    velo_1 = 0;
    velo_2 = 0;
    count_1 = 0;
    count_2 = 0;
  }
  else {
    if (flag_1 == 1){
      count_1 += 1;
      velo_1 += distPerCount_1/(timer_1-timer_1_0);
      timer_1_0 = timer_1;
      flag_1 = 0;
    }

    if (flag_2 == 1){
      count_2 += 1;
      velo_2 = distPerCount_2/(timer_2-timer_2_0);
      timer_2_0 = timer_2;
      flag_2 = 0;
    }
  }
}
