
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE 4

OneWire oneWire(ONE_WIRE);
DallasTemperature sensors(&oneWire);
DeviceAddress adresy[3] ={
{ 0x28, 0xB8, 0xEF, 0x15, 0x0D, 0x00, 0x00, 0xA4}, //levý
{ 0x28, 0xD3, 0x76, 0x17, 0x0D, 0x00, 0x00, 0xDC}, //střed
{ 0x28, 0xA5, 0x8C, 0x17, 0x0D, 0x00, 0x00, 0xED}
 }; //pravý

float teplota;

typedef struct{
  DeviceAddress address;
  int pin;
}jednotka;

jednotka levy, stred, pravy;


void zapisAdresu(jednotka *a, int j){
  Serial.println();
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("  ");
    Serial.print(adresy[j][i]);
    a->address[i]=adresy[j][i];
    Serial.print("  ");
    Serial.print(a->address[i]);
  }
  Serial.println();
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    //if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  sensors.begin();
  zapisAdresu(&levy, 0);
  levy.pin = 8;
  zapisAdresu(&stred, 1);
  stred.pin = 9;
  zapisAdresu(&pravy, 2);
  pravy.pin = 10;
  sensors.setResolution(levy.address, 12);
  sensors.setResolution(pravy.address, 12);
  sensors.setResolution(stred.address, 12);
  printAddress(levy.address);
  printAddress(stred.address);
  printAddress(pravy.address);
  pinMode(levy.pin, OUTPUT);
  pinMode(stred.pin, OUTPUT);
  pinMode(pravy.pin, OUTPUT);

}

void loop() {
  sensors.requestTemperatures();
  zkontroluj(levy);
  zkontroluj(stred);
  zkontroluj(pravy);
  Serial.println();
}

void zkontroluj(jednotka a){
  teplota = sensors.getTempC(a.address);
  Serial.print(teplota);
  Serial.print("\t");
  if(teplota>27.00){
   digitalWrite(a.pin, HIGH);
  }
  if(teplota<26.00){
   digitalWrite(a.pin, LOW);
  }
}
