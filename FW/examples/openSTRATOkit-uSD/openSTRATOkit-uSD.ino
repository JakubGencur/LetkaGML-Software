/*
   OpenSTRATOkit SD card read/write example


   @DevOK1RAJ Team, code by OK1CDJ 2/2021
  

 */

#include <SPI.h>
#include <SD.h>

File myFile;

void setup() {
  // Open Serial3 communications and wait for port to open:
  Serial3.pins(12, 13);
  Serial3.begin(115200);
  //pinMode(33,OUTPUT);
  
  SPI.pins(30, 31, 32, 33);
  
  digitalWrite(33,1);

  Serial3.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial3.println("initialization failed!");
    return;
  }
  Serial3.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial3.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial3.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial3.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial3.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial3.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial3.println("error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
}


