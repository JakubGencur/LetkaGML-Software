/*
   OpenSTRATOkit ATH10 Temperature and Humidity sensor

   @DevOK1RAJ Team, code by OK1CDJ 3/2021

   This example get data from onboard AHT10 sensor and print data to serial port

   Based on Adafruit ATH10 example

*/

#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

void setup() {
  Serial3.swap(1);
  Serial3.begin(115200);
  Serial3.println("OpenSTRATOkit AHT10 exmaple.");

  if (!aht.begin()) {
    Serial3.println("Could not find AHT? Check wiring");
    while (1)
      delay(10);
  }
  Serial3.println("AHT10 found");
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity,
               &temp); // populate temp and humidity objects with fresh data
  Serial3.print("Temperature: ");
  Serial3.print(temp.temperature);
  Serial3.println(" degrees C");
  Serial3.print("Humidity: ");
  Serial3.print(humidity.relative_humidity);
  Serial3.println("% rH");

  delay(1000);
}