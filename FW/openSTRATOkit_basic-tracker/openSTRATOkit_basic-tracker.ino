/*
   OpenSTRATOkit Basic Tracker


   @DevOK1RAJ Team, code by OK1CDJ 3/2021


  low ("space") frequency:     434.0 MHz
  frequency shift:             610 Hz
  baud rate:                   300 baud
  encoding:                    ASCII (7-bit)
  stop bits:                   2


  NOTE: RTTY frequency shift will be rounded
        to the nearest multiple of frequency step size.
        The exact value depends on the module:
        RF69 - 61 Hz steps

*/

// include the library
#include <Adafruit_AHTX0.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//#include <Adafruit_BME280.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <util/crc16.h>
#include <SPI.h>
#include <SD.h>

File myFile;


// Radio Settings

#define FREQ 434.690
#define SHIFT 610
#define BAUD 300
#define ENC ASCII
#define STOPB 2

String call = "RAJ10";
long pkt_num = 1;
float batt_voltage;

// RFM69W  has the following connections:
// NSS pin:   33
// DIO0 pin:  9
// RESET pin: RADIOLIB_NC

RF69 radio = new Module(33, 9, RADIOLIB_NC);

// create RTTY client instance using the FSK module
RTTYClient rtty(&radio);

// The TinyGPS++ object
TinyGPSPlus gps;

Adafruit_AHTX0 aht;
//Adafruit_BME280 bmp;

sensors_event_t humidity, temp;


void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setup() {
  Serial3.pins(12, 13);
  Serial3.begin(115200);
  Serial1.pins(18, 19);
  Serial1.begin(9600);
  //analogReference(INTERNAL1V1);

  Serial3.println("openSTRATOtracker");
  Serial3.println();
  // initialize AHT10 onboard sensor
  if (!aht.begin()) {
    Serial3.println("[AHT10] Could not find AHT10...");
    //while (true)
    //  ;
  } else  Serial3.println("[AHT10] Found...");
  // initialize RFM69 with default settings

  /* if(!bmp.begin())
  {
    Serial3.print("[BME280], no BME280 detected...");
   // while(1);
  } else Serial3.println("[BME280] found...");
  */

  Serial3.print(F("[RF69] Initializing ... "));
  SPI.pins(30, 31, 32, 33);
  int state = radio.begin();

  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    resetFunc();  
    while (true)
      ;
  }

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

  Serial3.print(F("[RTTY] Initializing ... "));

  state = rtty.begin(FREQ, SHIFT, BAUD, ENC, STOPB);
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    resetFunc();      
    Serial3.println(state);
    while (true)
      ;
  }
  Serial3.println(F("[GPS] Set flight mode ... "));
  //resetGPS();
  setGPS_DynamicModel6();
  delay(500);
  setGps_MaxPerformanceMode();
  delay(500); 

  SPI.pins(30, 31, 32, 33);
  
  digitalWrite(33,1);

  Serial3.print("[SD] Initializing SD card...");

  if (!SD.begin(10)) {
    Serial3.println("initialization failed!");
    //resetFunc();  
    //return;
  } else Serial3.println(F("success!"));

}

void loop() {
  // This sketch displays information every time a new sentence is correctly
  // encoded.
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());  
    //Serial3.println(gps.sentencesWithFix());
    //if (gps.encode(Serial1.read()))
    sendData();
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial3.println(F("No GPS detected."));
    resetGPS();
    resetFunc();  
    while (true)
      ;
  }
}

void sendData() {

  aht.getEvent(&humidity, &temp);
  //int press=bmp.readPressure();
  //bmp.getEvent(&eventBME);
  batt_voltage = (analogRead(A3) * 3.3 / 1024) *
                 ((15.0 + 33.0) / 33.0); // divider 15k and 33k
  //if (gps.location.isValid())
  if (gps.location.isUpdated() && gps.altitude.isUpdated())
  {
    String datastring;

    datastring = "$$$$" + call + ",";    // Call
    datastring += String(pkt_num) + ","; // Packet number
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
    datastring += String(gps.speed.mps()) + ",";          // speed
    datastring += String(gps.course.deg()) + ",";         // course
    datastring += String(batt_voltage, 2) + ",";          // voltage
    datastring += String(temp.temperature, 1) + ",";      // temperature internal
    //datastring += String(bmp.readTemperature(), 1) + ",";  // temperature external
    //datastring += String(bmp.readPressure(), 0) + ",";     // pressure
    datastring += String(gps.satellites.value());         // sats

    unsigned int CHECKSUM = gps_CRC16_checksum(datastring.c_str());
    char checksum_str[6];
    sprintf(checksum_str, "*%04X", CHECKSUM);
    datastring += String(checksum_str);

    Serial3.println(F("[RTTY] Sending RTTY data ... "));

    // send out idle condition for 500 ms
    rtty.idle();
    delay(1000);

    Serial3.println(datastring);
    rtty.println(datastring);

    Serial3.println(F("[RTTY] Done!"));
    writeData(datastring);
    pkt_num++;
  } 
}



void writeData(String Str)
{

   myFile = SD.open("data.txt", FILE_WRITE);
    if (myFile) {
    Serial3.print("[SD] Writing to data.txt...");
    myFile.println(Str);
    // close the file:
    myFile.close();
    Serial3.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial3.println("error opening data.txt");
  }


}
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
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

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
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
