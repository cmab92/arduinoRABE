#include <GP2D02.h>

#define OUT1 8
#define OUT2 9
#define IN1 6
#define IN2 7

int inputs[2] = {IN1,IN2};
int outputs[2] = {OUT1,OUT2};
nGP2D02 sensor(inputs,outputs,2);
const int delayMs = 100;

void setup()
{
  Serial.begin(57600);
};

void loop()
{
  for (int i = 0; i<2;i++) {
    int range = sensor.getRawRange(i+1);
    Serial.println(i);
    Serial.println(range);
  }
  delay(delayMs);
}
