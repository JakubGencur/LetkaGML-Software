//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Some neccessary variables for communication added
BluetoothSerial SerialBT;
String message = "";
char received;


void setup() {
  Serial.begin(115200);
  // Start bluetooth with visible name LetkaGMLRocket:
  SerialBT.begin("LetkaGMLRocket");
  // Send serial message and wait for device
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  // The next lines will be commented on Letka GML software:, served to send message from serial monitor to bluetooth connected phone
  //if (Serial.available()) {
  //  SerialBT.write(Serial.read());
  //}
  if (SerialBT.available()) { // If there is any availiable char in the buffer
    received = SerialBT.read(); // read unique character
    Serial.write(received); // write the character
    if(received < 41){ // if the character isn't readable (for man) the message is ended
      // START NEXT PHASE OF FLY, ADD CODE HERE:
      if(message == "data"){
        SerialBT.println("Start receiving data");
      }
      // reset message
      message = "";
      
    }
    // if the character is readable, ESP add it at the end of message
    else{
      message+=String(received);
    }
  }
  // for some reason wait 20 ms
  delay(20);
}
