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
#define WINDOWWIDTH 1 // width of evaluation window given as multiple if TSAMPLE

int count_1 = 0;
//float totalDistance_1 = 0;
int count_2 = 0;
//float totalDistance_2 = 0;

void count_isr_1() {
  count_1 += 1;
  //totalDistance_1 += 0.601/72;
}

void count_isr_2() {
  count_2 += 1;
  //totalDistance_2 += 0.602/72;
}

void setup() {
  Serial.begin(BAUDRATE);
  pinMode(IRPIN_1, INPUT);
  pinMode(IRPIN_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRPIN_1), count_isr_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IRPIN_2), count_isr_2, CHANGE);
}

void loop() {

  static unsigned long lastDistEval = micros();
  //static const float distPerCount_1 =  0.601/72;  // wheel circumference 0.601m, 72 steps/blocks
  //static const float distPerCount_2 =  0.602/72;  // wheel circumference 0.601m, 72 steps/blocks
  static unsigned int loopCount = 0;
  static float distance_1[WINDOWWIDTH];
  static float distance_2[WINDOWWIDTH];

  if ((micros()-lastDistEval) >= TSAMPLE){
    lastDistEval = micros();
    distance_1[loopCount] = count_1;//*distPerCount_1;
    distance_2[loopCount] = count_2;//*distPerCount_2;
    count_1 = 0;
    count_2 = 0;
    if ((loopCount<(WINDOWWIDTH-1)) && (loopCount>=0)){
      loopCount++;
    }
    else {
      loopCount = 0;
    }
    float v_1 = 0;
    float v_2 = 0;
    for (int i=0; i<WINDOWWIDTH; i++){
      v_1 += distance_1[i];
      v_2 += distance_2[i];
    }
    //v_1 = v_1/(WINDOWWIDTH*TSAMPLE_S);
    //v_2 = v_2/(WINDOWWIDTH*TSAMPLE_S);
    Serial.print(v_1);
    Serial.print(", ");
    Serial.print(v_2);
    Serial.print(", ");
    Serial.print("");  // dummy
    Serial.print(", ");
    Serial.print("");  // dummy
    Serial.print("\n");
  }
}
