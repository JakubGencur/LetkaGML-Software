/*
  openSTRATOkit Basic Tracker

  @DevOK1RAJ Team, code by OK1CDJ 3/2021

  low ("space") frequency:     434.69 MHz
  frequency shift:             610 Hz
  baud rate:                   300 baud
  encoding:                    ASCII (7-bit)
  stop bits:                   2

  NOTE: RTTY frequency shift will be rounded
        to the nearest multiple of frequency step size.
        The exact value depends on the module:
        RF69 - 61 Hz steps
*/

// libraries
#include <Adafruit_Sensor.h>
//#include <Wire.h>
//#include <SD.h>

//########libraries - Letka GML#######

//dallas (temperature)
#include <OneWire.h>
#include <DallasTemperature.h>

//ccs811 (CO2)       https://joy-it.net/en/products/SEN-CCS811V1
#include "Adafruit_CCS811.h"

//mpu9250  (accelerometer, gyroscope, magnetometr)
#include "MPU9250.h"

//UVA, UVB

//################end#################

//###Letka###
Adafruit_CCS811 co2sensor;

MPU9250 ac_gy_mag(Wire,0x68);
int status;

OneWire Bus(4);
DallasTemperature Temperature(&Bus);
DeviceAddress DevAdr;
int nSensors;
//####end####

//File myFile;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setup() {

  // init serial comm
  Serial.begin(115200);
  Serial.println("openSTRATOtracker");
  Serial.println();
  //###########Letka's senzors initialization:#######
  
  //Dallas:
  Serial.print("[Dallas(temperature)] Initializing Dallas senzors...");
  Temperature.begin();
  nSensors = Temperature.getDeviceCount();

  
  //ccs811:
  if(!co2sensor.begin()){
    Serial.println("[CCS811(CO2)] Initializing CCS811 senzor..."); 
  }
  
  //MPU9250:
  if(!ac_gy_mag.begin()){
    Serial.println("[MPU9250(ac, gy, mag)] Initializing MPU9250 senzor...");
  }
  
  //UVA/UVB:
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  
  Serial.println("end of setup");
  
  //#######################end#######################
}

void loop() {
  sendData();
}
  
// compile sensor information and transmit it
void sendData() {
    String datastring_Letka;
    Temperature.requestTemperatures();
    for (uint8_t i = 0; i < nSensors; i++){
      //datastring_Letka += String(i + 1);
      //datastring_Letka += ":";
      datastring_Letka += String(Temperature.getTempCByIndex(i)) + ",";
    }
    if(co2sensor.available()){
      if(co2sensor.readData()){
        //datastring_Letka += "CO2: ";
        datastring_Letka += String(co2sensor.geteCO2());
        //datastring_Letka += "/*ppm, TVOC:";
        datastring_Letka += ",";
        datastring_Letka += String(co2sensor.getTVOC());
      }
      else{
        datastring_Letka += "n,n,";
      }
    }
    ac_gy_mag.readSensor();
    // display the data
    datastring_Letka += String(ac_gy_mag.getAccelX_mss());
    datastring_Letka += ",";
    datastring_Letka += String(ac_gy_mag.getAccelY_mss());
    datastring_Letka += ",";
    datastring_Letka += String(ac_gy_mag.getAccelZ_mss());
    datastring_Letka += ",";
    datastring_Letka += String(ac_gy_mag.getGyroX_rads());
    datastring_Letka += ",";
    datastring_Letka += String(ac_gy_mag.getGyroY_rads());
    datastring_Letka += ",";
    datastring_Letka += String(ac_gy_mag.getGyroZ_rads());
    datastring_Letka += ",";
    datastring_Letka += String(ac_gy_mag.getMagX_uT());
    datastring_Letka += ",";
    datastring_Letka += String(ac_gy_mag.getMagY_uT());
    datastring_Letka += ",";
    datastring_Letka += String(ac_gy_mag.getMagZ_uT());
    datastring_Letka += ",";
    datastring_Letka += String(ac_gy_mag.getTemperature_C());
    
    datastring_Letka += ",";
    datastring_Letka += String(getUV());


    Serial.println(datastring_Letka);
    //writeData(datastring_Letka); // write a copy to the SD card 
}

// write data to the SD card
/*void writeData(String Str) {
   myFile = SD.open("data.txt", FILE_WRITE);
   if (myFile) {
    Serial.print("[SD] Writing to data.txt...");
    myFile.println(Str);
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening data.txt");
  }
}*/




float getUV(){
	int valt=manalogRead(A1);
	int val=manalogRead(A0);
	float tension=3.3 / valt * val;
	return (val - 0.96) * (15.0 - 0.0) / (2.8 - 0.96) + 0.0;
}

int manalogRead(int pin){
	int v = 0;
	for(int x=0; x<8;  x++){
		v+=analogRead(pin);
	}
	v/=8;
	return v;
}
