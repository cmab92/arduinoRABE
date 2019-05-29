#include <Arduino.h>
#include <PulseInZero.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

const int AnzahlWerte = 10;
String strEingabe;
int werte [AnzahlWerte];
int kommas [AnzahlWerte+1];
boolean bNewdata = false;

//Akku und LED variablen---------------
int akku=0; //    variable für akkumesswert
int akkumitte=610; //schwellwert für mittleren ladezustand
int akkuleer=590; //schwellwert für leeren ladezustand
int gruenpwm=50; // pwmwert 0 bis 255 für grünanteil
int rotpwm=70;  // pwmwert 0 bis 255 für rotanteil
int akkustop=0; //bei 1 wird fahren deaktiviert, bei 0 fahren möglich

//Pinbelegung--------------------
int led1G=10;
int led1R=11;
int led2G=5;
int led2R=6;
int transistor=4; // Transistor an Pin D4
int lenksensor=7; // Graues Kabel zum Lenksensor für Lenkwinkelnullpunkt Sinal = LOW sonst HIGH
int hall_1=2; //erster Hallsensor für Geschwindigkeit
int hall_2=3; //zweiter Hallsensor für Drehrichtung
int taster1=15; //taster1 an D15 bzw A1
int akkupin=A6; //akkuspannung an A6 Prüfen!!!

//Zeit und geschwindigkeits variablen--------------------------------------
unsigned long time=0;
unsigned long halldauer=0;
unsigned long powertime=0;
unsigned long powertimeout=120000; //120 Sekunden

//sonstige Variablen----------------------------------------------------
int richtung;
int gasminaus=800;
int gas=1250; // maximal ca 3200 -> ca 4 Volt für ebike controller
int taster=0;
//---------------------------------------------------------------------------------

void akkustand()
{
	akku=analogRead(akkupin);// Akku voll Grün
	if (akku>akkumitte)
	{
	  analogWrite(led1G,gruenpwm);
	  analogWrite(led2G,gruenpwm);
	  analogWrite(led1R,0);
	  analogWrite(led2R,0);
	  akkustop=0;
	}

	if (akku<=akkumitte&&akku>akkuleer)// Akku<=mitte & akku>leer -> Gelb
	{
	  analogWrite(led1G,gruenpwm);
	  analogWrite(led2G,gruenpwm);
	  analogWrite(led1R,rotpwm);
	  analogWrite(led2R,rotpwm);
	  akkustop=0;
	}

	if (akku<=akkuleer)
	{
	  analogWrite(led1G,0);
	  analogWrite(led2G,0);
	  analogWrite(led1R,rotpwm);
	  analogWrite(led2R,rotpwm);
	  akkustop=1;
	}
}
//----------------------------------------------------------------

void gastaster()
{
   int _taster1=digitalRead(taster1);

  if (_taster1==LOW) //taster1&2 abfragen (mit pullup, d.h. gedrückt = LOW)
    {
		taster=1;
    }
  else //wenn beide HIGH sind alles deaktivieren
    {
		taster=0;
	}

}

void hallinterrupt(unsigned long duration)
{
	halldauer = duration; //duration wird vom interrupt uebergeben: pulse length in microseconds
	richtung = digitalRead(hall_2);
	/* Start listening again... Sobald der interrupt einmal gecalled wird
	ist der interrupt vorerst deaktiviert und muss manuell wieder
	aktiviert werden. */
	PulseInZero::begin();
  }

void setup()
{
  Serial.begin(115200);
  PulseInZero::setup(hallinterrupt);
  PulseInZero::begin(); //nur zur Erinnerung: PulseInZero::abandon();

  dac.begin(0x62);
  dac.setVoltage(0, false);

  pinMode(led1G, OUTPUT); // Grüne LED1
  pinMode(led1R, OUTPUT); // Rote LED1
  pinMode(led2G, OUTPUT); // Grüne LED2
  pinMode(led2R, OUTPUT); // Rote LED2
  pinMode(hall_1, INPUT); // Halleingang1 D2
  pinMode(hall_2, INPUT); // Halleingang2 D3
  pinMode(lenksensor, INPUT); // Lenksensor auf D7 (Bei Rabe ungenutzt, da Lenkbares Rad mit Sensor)
  pinMode(taster1, INPUT_PULLUP); //Taster1 eingang (A1/15)
  pinMode(transistor, OUTPUT); //Transistor für Selbsthalteschaltung
  digitalWrite(transistor, HIGH); //Halteschaltung aktivieren
  kommas[0] = -1;

}

void loop()
{

    if (Serial.available()){
     strEingabe=Serial.readString();
     bNewdata = true;
 }
 if (bNewdata) { // Wird nur ausgeführt wennn neue Daten vorliegen!
    bNewdata = false;
    for(int i=0;i<AnzahlWerte;i++){
      kommas[i+1] = strEingabe.indexOf(",",kommas[i]+1);
      werte[i] = strEingabe.substring(kommas[i]+1,kommas[i+1]).toInt();
      }
      gas=werte[0]; //beliebig erweitern...
   }

   Serial.print(halldauer);
   Serial.print(",");
   Serial.print(gas);
   Serial.print(",");
   Serial.println("");//beliebig erweitern...

time=millis();

gastaster();

if ((time-powertime)>=powertimeout)
	{
		digitalWrite(transistor,LOW); //schaltet sich selbst ab
	}

if (taster==0)//wenn gas nicht betätigt oder lenkwinkel im verbotenen bereich ist
  {
	  dac.setVoltage(gasminaus, false);
	  akkustand();
  }


if (taster==1) //bei tasten druck oder schieben im pedelec timer zurücksetzen
{
	powertime=time; //aktuelle zeit speichern für timeout
	digitalWrite(transistor, HIGH); //Halteschaltung aktivieren
	dac.setVoltage(gas, false);
}
dac.setVoltage(2000, true);

}//loopende

//Funktionen----------------------------------------------------------------
// #include <Arduino.h>
// const int dirPin = 4;
// const int stepPin = 5;
// const int sleepPin = 6;
// const int resetPin = 7;
// const int enablePin = 8;
// const int m0Pin = 9;
// const int m1Pin = 10;
// const int m2Pin = 11;
// const int drehgeberPin=14; //(A0)
// const int hallsensor=15; //(A1)
//
// int steps=2000;
//
// int pause=500; //delay in µs je Step (500)
// int drehgeber;
//
// void setup() {
//   Serial.begin(9600);
//
//   pinMode(stepPin,OUTPUT);
//   pinMode(dirPin,OUTPUT);
//   pinMode(sleepPin,OUTPUT);
//   pinMode(resetPin,OUTPUT);
//   pinMode(enablePin,OUTPUT);
//   pinMode(hallsensor,INPUT_PULLUP);
//
//   // Treiber aktivieren
//   digitalWrite(sleepPin,HIGH); // HIGH für aktivierung des Treibers
//   digitalWrite(resetPin,HIGH); // HIGH für aktivierung des Treibers
//   //digitalWrite(enablePin,HIGH); //The default state of the ENBL pin is to enable the driver, so this pin can be left disconnected.
//
//   //microstepping Treiber: DRV8825
//   digitalWrite(m0Pin,HIGH); // Low/Low/Low=Full step ; High/Low/Low=Half step ; Low/High/Low=1/4 step;
//   digitalWrite(m1Pin,LOW);  // High/High/Low=1/8 step ; Low/Low/High=1/16 step; High/Low/High=1/32 step;
//   digitalWrite(m2Pin,HIGH);
// }
//
// void loop() {
//   drehgeber=analogRead(drehgeberPin);
//   Serial.print("drehgeber: ");
//   Serial.println(drehgeber);
//
//   //zum testen
//   digitalWrite(dirPin,HIGH); // Rechts drehung (im Uhrzeigersinn in draufsicht)
//   for(int x = 0; x < steps; x++) {
//     digitalWrite(stepPin,HIGH);
//     delayMicroseconds(pause);
//     digitalWrite(stepPin,LOW);
//     delayMicroseconds(pause);
//   }
//
//   digitalWrite(dirPin,LOW); // Links drehung (gegen Uhrzeigersinn in draufsicht)
//   for(int x = 0; x < steps; x++) {
//     digitalWrite(stepPin,HIGH);
//     delayMicroseconds(pause);
//     digitalWrite(stepPin,LOW);
//     delayMicroseconds(pause);
//   }
//
//
//
// }
