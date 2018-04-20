
/*
cb, 20.04.18
*/
#include <TimerOne.h>

#define TSAMPLE 16000 //in us (16500us := measuring cycle of 0a41sk/0a51sk/0a21ysk/0a02yk )
#define NOS 12 // number of sensors

void measure( void ) {
  for (int i = 0; i<NOS; i++){
    analogRead(i); // dummy measurement for adc
    Serial.print( analogRead(i) );
    Serial.print( "," );
  }
  Serial.print("\n");
}

void setup() {
  Serial.begin(57600);

  Timer1.initialize(TSAMPLE);
  Timer1.attachInterrupt(measure);
}

void loop() {
  while(1){
      // wait for interrupt (respectively next sample)
  }
}
