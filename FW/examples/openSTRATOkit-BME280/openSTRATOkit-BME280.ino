/*
   OpenSTRATOkit BME280 sensor example

   @DevOK1RAJ Team, code by oktkas 5/2021

   uses Adafruit BME280 library
    
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
   
Adafruit_BME280 bme;

void setup() {

  // set up serial comm
  Serial3.pins(12, 13);
  Serial3.begin(115200);
  Serial3.println("BME280 test"); Serial3.println("");
  
  // init and check for BME280
  if(!bme.begin(0x76)){
    Serial3.print("[BME280], no BME280 detected...");
  } else Serial3.println("[BME280] found...");
}

void loop() {
  // print the values every 2 seconds
  Serial3.print("Temperature = ");
  Serial3.print(bme.readTemperature());
  Serial3.println(" °C");

  Serial3.print("Pressure = ");

  Serial3.print(bme.readPressure() / 100.0F);
  Serial3.println(" hPa");

  Serial3.print("Approx. Altitude = ");
  Serial3.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial3.println(" m");

  Serial3.print("Humidity = ");
  Serial3.print(bme.readHumidity());
  Serial3.println(" %");

  Serial3.println();
  delay(2000);
}
