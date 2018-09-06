/*
Author: cb
Date: April 18
Purpose:
 - read analog inputs (ir-data and force sensors)
 - read bno data
 - write data to serial
 - gray coding analog data (?)
 - write calibration registers of bno
 -
Attention:
 - no high acceleration interrupt of bno (!)
 - check BAUDRATE
 - check TSAMPLE
 - check NOI
 - sda and scl on pin 20 / 21
 - supply voltage for voltage divider of force sensor on pin 2 (R2 = 15)
 - only for atmega2560
 - no interrupt control (conflict with i2c)
 - quaternion data not normalized ( normalize by 16384 = 2¹⁴ LSB)
 -
*/

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
