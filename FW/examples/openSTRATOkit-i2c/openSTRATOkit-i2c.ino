/*
   OpenSTRATOkit I2C scanner

   @DevOK1RAJ Team, code by OK1CDJ 12/2020 
    
   This example tries to find I2C Devices on connected to the KIT
*/

#include <Wire.h>

void setup()
{
  Wire.begin();
  Serial3.swap(1);
  Serial3.begin(115200);
  Serial3.println("\nI2C Scanner");
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial3.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial3.print("I2C device found at address 0x");
      if (address<16) 
        Serial3.print("0");
      Serial3.print(address,HEX);
      Serial3.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial3.print("Unknown error at address 0x");
      if (address<16) 
        Serial3.print("0");
      Serial3.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial3.println("No I2C devices found\n");
  else
    Serial3.println("done\n");

  delay(5000); // wait 5 seconds for next scan
}
