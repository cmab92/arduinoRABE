#include <Arduino.h>

void setup() {
    Serial.begin(57600);
    Serial.println("...");
}

void loop() {
  int meas = analogRead(0);
  Serial.println(meas);
  delay(50);
    // put your main code here, to run repeatedly:
}
