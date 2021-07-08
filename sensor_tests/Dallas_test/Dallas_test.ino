
//Dallas na dratku

#include <OneWire.h>
#include <DallasTemperature.h>

#define BUS_PIN 4
#define ADR_SIZE 8 //byte

//vytvoreni instanci
OneWire Bus(BUS_PIN);
DallasTemperature Sensors(&Bus);
DeviceAddress DevAdr;

uint8_t nSensors = 0;

void printAddress(DeviceAddress devAdr, uint8_t idx);
void getTemp();

void printAddress(DeviceAddress devAdr, uint8_t idx)
{
  Serial.print("Adresa senzoru ");
  Serial.print(idx + 1);
  Serial.print(": ");

  for (uint8_t i = 0; i < ADR_SIZE; i++)
  {
    if (devAdr[i] < 16) Serial.print("0");
    Serial.print(devAdr[i], HEX);
  }
  Serial.println();
}

void getTemp(){
  Sensors.requestTemperatures();

  for (uint8_t i = 0; i < nSensors; i++)
  {
    Serial.print("Teplota cidla ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(Sensors.getTempCByIndex(i));
  }
  Serial.println("****************************");
  delay(2000);
}

void setup(void) {
  Serial.begin(9600);
  Sensors.begin();

  //pocet senzoru
  nSensors = Sensors.getDeviceCount();

  //vypis poctu senzoru
  Serial.print("Pocet senzoru: ");
  Serial.println(nSensors);

  //vypis adres senzoru
  for (uint8_t i = 0; i < nSensors; i++)
  {
    Sensors.getAddress(DevAdr, i);
    printAddress(DevAdr, i);
  }
  Serial.println("****************************");
}

void loop(void) {
  //cteni teploty
  getTemp();
}
