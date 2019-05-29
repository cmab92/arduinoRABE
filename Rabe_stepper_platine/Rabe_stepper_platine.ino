const int dirPin = 4;
const int stepPin = 5; 
const int sleepPin = 6;
const int resetPin = 7;
const int enablePin = 8;
const int m0Pin = 9;
const int m1Pin = 10;
const int m2Pin = 11;
const int drehgeberPin=14; //(A0)
const int hallsensor=15; //(A1)

int steps=2000;

int pause=500; //delay in µs je Step (500)
int drehgeber;

void setup() {
  Serial.begin(9600);

  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(sleepPin,OUTPUT);
  pinMode(resetPin,OUTPUT);
  pinMode(enablePin,OUTPUT);
  pinMode(hallsensor,INPUT_PULLUP);

  // Treiber aktivieren
  digitalWrite(sleepPin,HIGH); // HIGH für aktivierung des Treibers
  digitalWrite(resetPin,HIGH); // HIGH für aktivierung des Treibers
  //digitalWrite(enablePin,HIGH); //The default state of the ENBL pin is to enable the driver, so this pin can be left disconnected.

  //microstepping Treiber: DRV8825
  digitalWrite(m0Pin,HIGH); // Low/Low/Low=Full step ; High/Low/Low=Half step ; Low/High/Low=1/4 step;
  digitalWrite(m1Pin,LOW);  // High/High/Low=1/8 step ; Low/Low/High=1/16 step; High/Low/High=1/32 step;
  digitalWrite(m2Pin,HIGH);
}

void loop() { 
  drehgeber=analogRead(drehgeberPin);
  Serial.print("drehgeber: ");
  Serial.println(drehgeber);

  //zum testen
  digitalWrite(dirPin,HIGH); // Rechts drehung (im Uhrzeigersinn in draufsicht)
  for(int x = 0; x < steps; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(pause); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(pause); 
  }

  digitalWrite(dirPin,LOW); // Links drehung (gegen Uhrzeigersinn in draufsicht)
  for(int x = 0; x < steps; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(pause);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(pause);
  }



}


