/*
Author: cb
Date: April 18
Purpose:
 - read analog inputs (ir-data and force sensors)
 - write data to serial
 -
Attention:
 - check BAUDRATE
 - check TSAMPLE
 - check NOI
 -
*/

 #include "Arduino.h"
 #include <Wire.h>
// #include <Adafruit_BNO055.h>

#define TSAMPLE 16496 //in us (16500us := measuring cycle of 0a41sk/0a51sk/0a21ysk/0a02yk )
#define NOI 8 //12  // number of analog inputs (12 IR + 2 iefsr-force-sensors)
#define BAUDRATE 57600
#define VDDSUPPLY1 11
#define VDDSUPPLY2 12
#define WWIDTH 32

void measure( void ) {
  int val[NOI];
  for (int i = 0; i<NOI; i++){
    val[i] = 0;
  }
  for (int j = 0; j<WWIDTH; j++){
    for (int i = 0; i<NOI; i++){
      // analogRead(i); // dummy measurement for adc
      val[i] += analogRead(i);
      }
  }
  for (int i = 0; i<NOI; i++){
    Serial.print(val[i]/WWIDTH);
    Serial.print(",");
  }
  Serial.print("\n");
}

void setup(){
  Serial.begin(BAUDRATE);
  delay(20);
  pinMode(VDDSUPPLY1, OUTPUT);
  pinMode(VDDSUPPLY2, OUTPUT);
}

void loop() {

  digitalWrite(VDDSUPPLY1, HIGH);
  digitalWrite(VDDSUPPLY2, HIGH);
  unsigned long lastMeasurement = micros();
  measure();

  while (1) {
    if ((micros()-lastMeasurement) >= TSAMPLE){
      // Serial.println(micros()-lastMeasurement);
      lastMeasurement = micros();
      measure();
    }
  }
}
