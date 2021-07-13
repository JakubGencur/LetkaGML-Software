/*
  openSTRATOkit Basic Tracker

  @DevOK1RAJ Team, code by OK1CDJ 3/2021

  code improved by Letka GML (added code some others sensors)

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
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <util/crc16.h>
#include <SPI.h>
#include <SD.h>

//########libraries - Letka GML#######

//dallas (temperature)
#include <OneWire.h>
#include <DallasTemperature.h>

//ccs811 (CO2)       https://joy-it.net/en/products/SEN-CCS811V1
#include "Adafruit_CCS811.h"

//mpu9250  (accelerometer, gyroscope, magnetometr)
#include "MPU9250.h"

//ml8511  (uva, uvb)

//################end#################

// Radio Settings
#define FREQ 434.690
#define SHIFT 610
#define BAUD 300
#define ENC ASCII
#define STOPB 2

String call = "TTS10"; // CHANGE THIS!
long pkt_num = 1; // packet number
float batt_voltage;

// create File variable for SD card
File myFile;

// create radio module variable
RF69 radio = new Module(33, 9, RADIOLIB_NC);

// create RTTY client instance using the FSK module
RTTYClient rtty(&radio);

// create gps clien
TinyGPSPlus gps;

// create barometric sensor client
Adafruit_BME280 bme;

//###Letka###

// create CO2 sensor client
Adafruit_CCS811 co2senzor;

// create megnetometer/accelerometer/gyroscope client
MPU9250 ac_gy_mag(Wire, 0x68);

int status;

//create temperature client and initialize Adress variable (DevAdr)
uint8_t nSensorsTemperature;
OneWire Bus(11);
DallasTemperature Temperature(&Bus);
DeviceAddress DevAdr;

//####end####

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setup() {

  // init serial comm  (Serial1 = gps module, Serial3 = usb)
  Serial3.pins(12, 13);
  Serial3.begin(115200);
  Serial1.pins(18, 19);
  Serial1.begin(9600);
  Serial3.println("openSTRATOtracker");
  Serial3.println();

  // init and check for BME280
  if(!bme.begin(0x76)){
    Serial3.print("[BME280], no BME280 detected...");
  } else Serial3.println("[BME280] found...");
  
  // init radio
  Serial3.print(F("[RF69] Initializing ... "));
  SPI.pins(30, 31, 32, 33);
  int state = radio.begin();

  // check for errors (caused in radio init)
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    resetFunc();  
    while (true);
  }

  // radio output power
  Serial3.print(F("[RF69] Setting high power module ... "));
    state = radio.setOutputPower(20, true);
    if (state == ERR_NONE) {
      Serial3.println(F("success!"));
    } else {
      Serial3.print(F("failed, code "));
      Serial3.println(state);
      resetFunc();  
      while (true);
    }

  // set-up rtty comm
  Serial3.print(F("[RTTY] Initializing ... "));
  state = rtty.begin(FREQ, SHIFT, BAUD, ENC, STOPB);
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    resetFunc();      
    Serial3.println(state);
    while (true);
  }

  // set-up GPS
  Serial3.println(F("[GPS] Set flight mode ... "));
  setGPS_DynamicModel6();
  delay(500);
  setGps_MaxPerformanceMode();
  delay(500); 
  
  digitalWrite(33,1);

  // init SD card
  Serial3.print("[SD] Initializing SD card...");
  if (!SD.begin(10)) {
    Serial3.println("initialization failed!");
  } else Serial3.println(F("success!"));
  
  //###########Letka's senzors initialization:#######
  
  //Dallas:
  Serial3.print("[Dallas(temperature)] Initializing Dallas senzors...");
  Temperature.begin();
  nSensorsTemperature = Temperature.getDeviceCount();
  Serial3.print(nSensorsTemperature);
  Serial3.print(" sensors found...");
  for (uint8_t i = 0; i < nSensorsTemperature; i++)
  {
    Temperature.getAddress(DevAdr, i);
  }
  Serial3.println(F("success!"));
  
  //ccs811:
  Serial3.println("[CCS811(CO2)] Initializing CCS811 senzor...");
  if(!co2senzor.begin()){
    while(1);
  } else Serial3.println(F("success!"));
  
  //MPU9250:
  Serial3.println("[MPU9250(ac, gy, mag)] Initializing MPU9250 senzor...");
  if(!ac_gy_mag.begin()){
    while(1);
  } else Serial3.println(F("success!"));
  
  
  //ML8511:
  Serial3.println("[ML8511(UVA/UVB)] Initializing ML8511 senzor...");
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  Serial3.println(F("success!"));
  
  //#######################end#######################
}

void loop() {

  // read GPS and send RTTY messages
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
    sendData();
  }

  // try to fix the GPS in case of a malfunction
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial3.println(F("No GPS detected."));
    resetGPS();
    resetFunc();  
    while (true);
  }
}

// compile sensor information and transmit it
void sendData() {

  // calculate battery voltage
  batt_voltage = (analogRead(A3) * 3.3 / 1024) *
                 ((15.0 + 33.0) / 33.0); // divider 15k and 33k

  if (gps.location.isUpdated() && gps.altitude.isUpdated()) {
    String datastring;

    datastring = "$$$$" + call + ",";    // Call
    datastring += String(pkt_num) + ","; // Packet number

    // GPS time
    if (gps.time.hour() < 10)
      datastring += "0" + String(gps.time.hour()) + ":";
    else
      datastring += String(gps.time.hour()) + ":";

    if (gps.time.minute() < 10)
      datastring += "0" + String(gps.time.minute()) + ":";
    else
      datastring += String(gps.time.minute()) + ":";

    if (gps.time.second() < 10)
      datastring += "0" + String(gps.time.second()) + ",";
    else
      datastring += String(gps.time.second()) + ",";

    datastring += String(gps.location.lat(), 6) + ",";    // lat
    datastring += String(gps.location.lng(), 6) + ",";    // long
    datastring += String(gps.altitude.meters(), 0) + ","; // altitude
    //datastring += String(gps.speed.mps()) + ",";          // speed
    //datastring += String(gps.course.deg()) + ",";         // course
    datastring += String(batt_voltage, 2) + ",";          // voltage
    //datastring += String(temp.temperature, 1) + ",";      // temperature internal
    datastring += String(bme.readTemperature(), 1) + ",";  // temperature external
    datastring += String(bme.readPressure(), 0) + ",";     // pressure
    datastring += String(gps.satellites.value());         // sats
    String datastring_Letka = datastring;
    datastring_Letka += ", temps: ";
    Temperature.requestTemperatures();
    for (uint8_t i = 0; i < nSensorsTemperature; i++){
      //datastring_Letka += String(i + 1);
      //datastring_Letka += ":";
      datastring_Letka += String(Temperature.getTempCByIndex(i)) + ",";
    }
    if(co2senzor.available()){
      if(!co2senzor.readData()){
        datastring_Letka += String(co2senzor.geteCO2());
        datastring_Letka += ",";
        datastring_Letka += String(co2senzor.getTVOC());
        datastring_Letka += ",";
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

    // checksum
    unsigned int CHECKSUM = gps_CRC16_checksum(datastring.c_str());
    char checksum_str[6];
    sprintf(checksum_str, "*%04X", CHECKSUM);
    datastring += String(checksum_str);

    // transmit the data
    Serial3.println(F("[RTTY] Sending RTTY data ... "));

    // send out idle condition for 500 ms
    rtty.idle();
    delay(1000);

    Serial3.println(datastring_Letka);
    rtty.println(datastring);

    Serial3.println(F("[RTTY] Done!"));
    writeData(datastring_Letka); // write a copy to the SD card
    pkt_num++; //advance packet number
  } 
}

// write data to the SD card
void writeData(String Str) {
   myFile = SD.open("data.txt", FILE_WRITE);
   if (myFile) {
    Serial3.print("[SD] Writing to data.txt...");
    myFile.println(Str);
    myFile.close();
    Serial3.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial3.println("error opening data.txt");
  }
}

// calculate a CRC16 checksum
uint16_t gps_CRC16_checksum(char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first four $s
  for (i = 4; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update(crc, c);
  }
  return crc;
}


void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial1.flush();
  Serial1.write(0xFF); //
  delay(100);
  for(int i=0; i<len; i++) {
    Serial1.write(MSG[i]);
  }
}

void resetGPS() {
  /*
  Forced (Watchdog)
  Coldstart
  */
  uint8_t set_reset[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5};
  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}

void setGPS_DynamicModel6()
{
  /*
  CFG-NAV5

Header: 0xB5, 0x62, 
ID: 0x06, 0x24, 
Length 0x24, 0x00, 
mask 0xFF, 0xFF, 
dynModel:  0x06, (Airborne <1g)
fixMode: 0x03, 
fixedAlt: 0x00, 0x00, 0x00, 0x00, 
fixedAltVar: 0x10, 0x27, 0x00, 0x00, 
minElev 0x05, 
drLimit 0x00, 
pDop 0xFA, 0x00, 
tDop 0xFA, 0x00, 
pAcc 0x64, 0x00, 
tAcc 0x2C, 0x01, 
staticHoldThresh 0x00, 
dgpsTimeOut 0x00, 
0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00,
CK_A 0x16, 
CK_B 0xDC  
  
  */
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                                                             };
  while(!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
    //morse("OK");
}

void setGps_MaxPerformanceMode() {
  /*
  UBX-CFG-RMX - 0 Continuous Mode (Max Performance Mode)
  */
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91                                                                             }; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial1.available()) {
      b = Serial1.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }

    }
  }
}

float getUV(){
	int FvF=manalogRead(A1);
	int val=manalogRead(A0);
	float tension=3.3 / FvF * val;
	return (tension - 0.96) * (15.0 - 0.0) / (2.8 - 0.96) + 0.0;
}

int manalogRead(int pin){
	int v = 0;
	for(int x=0; x<8; x++){
		v+=analogRead(pin);
	}
	v/=8;
	return v;
}
