/*
   OpenSTRATOkit GPS example

   @DevOK1RAJ Team, code by ok1cdj 5/2021

   uses TinyGPS++ library
    
*/


#include <TinyGPS++.h>

// The TinyGPS++ object
TinyGPSPlus gps;

void setup() {
  // Serial comm to the PC
  Serial3.pins(12, 13);
  Serial3.begin(115200);

  // Serial comm to the GPS
  Serial1.pins(18, 19);
  Serial1.begin(9600);

  Serial3.println(F("GPS TEST"));
  Serial3.println();
}

void loop() {
  // print info if possible
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      displayInfo();

  // check if the module is alive
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial3.println(F("No GPS detected: check wiring."));
    while (true);
  }
}

// print GPS info in a human friendly format
void displayInfo() {
  Serial3.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial3.print(gps.location.lat(), 6);
    Serial3.print(F(","));
    Serial3.print(gps.location.lng(), 6);
    Serial3.print(F(" SAT: "));
    Serial3.print(gps.satellites.value(), 6);
  }
  else {
    Serial3.print(F("INVALID"));
  }

  Serial3.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial3.print(gps.date.month());
    Serial3.print(F("/"));
    Serial3.print(gps.date.day());
    Serial3.print(F("/"));
    Serial3.print(gps.date.year());
  }
  else {
    Serial3.print(F("INVALID"));
  }

  Serial3.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial3.print(F("0"));
    Serial3.print(gps.time.hour());
    Serial3.print(F(":"));
    if (gps.time.minute() < 10) Serial3.print(F("0"));
    Serial3.print(gps.time.minute());
    Serial3.print(F(":"));
    if (gps.time.second() < 10) Serial3.print(F("0"));
    Serial3.print(gps.time.second());
    Serial3.print(F("."));
    if (gps.time.centisecond() < 10) Serial3.print(F("0"));
    Serial3.print(gps.time.centisecond());
  }
  else{
    Serial3.print(F("INVALID"));
  }

  Serial3.println();
}
