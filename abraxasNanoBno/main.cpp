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


void measure( void ) {
  for (int i = 0; i<NOI; i++){
    analogRead(i); // dummy measurement for adc
    Serial.print( analogRead(i) );
    Serial.print( "," );
  }
  Serial.print("\n");
}

void setup(){
  Serial.begin(BAUDRATE);
  delay(50);
}

void loop() {

  unsigned long lastMeasurement = micros();
  measure();

  while (1) {
    if ((micros()-lastMeasurement) >= TSAMPLE){
      lastMeasurement = micros();
      measure();
      // Serial.println(micros()-lastMeasurement);
    }
  }
}
