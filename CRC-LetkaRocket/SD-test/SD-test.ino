/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       12
 *    D3       13
 *    CMD      15
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      14
 *    VSS      GND
 *    D0       2  (add 1K pull up after flashing)
 *    D1       4
 */

#include "FS.h"
#include "SD_MMC.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

const char * file_name = "/data.txt";
File file;

BluetoothSerial SerialBT;
String BTmessage = "";
int i=0;


void BT_SD_print(const char * message){
    file.print(message);
    SerialBT.println(message);
}
void Serial_SD_BT_print(String message){
    file.print(message);
    Serial.println(message);
    SerialBT.println(message);
}

void setup(){
    Serial.begin(115200);

    SerialBT.begin("LetkaGMLRocket");
    
    if(!SD_MMC.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD_MMC.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD_MMC card attached");
        return;
    }

    Serial.print("SD_MMC Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

    file = SD_MMC.open(file_name, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    const char* mess = "new writing";
    file.println(mess);
    file.close();
}

void loop(){
  Serial_SD_BT_print("Hello world!");
  delay(500);
  i++;
  if(i%10 == 0){
    file.close();
    delay(10);
    file = SD_MMC.open(file_name, FILE_WRITE);
  }

}
