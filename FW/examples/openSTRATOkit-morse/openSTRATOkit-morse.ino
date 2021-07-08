/*
   OpenSTRATOkit Morse Transmit Example

   @DevOK1RAJ Team, code by OK1CDJ 12/2020 

   This example sends Morse code message using
   RFM69 FSK modem and RadioLib.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

// Radio Settings

#define FREQ 434.000

// RFM69HW  has the following connections:
// NSS pin:   33
// DIO0 pin:  9
// RESET pin: RADIOLIB_NC

RF69 radio = new Module(33, 9, RADIOLIB_NC);

// create Morse client instance using the FSK module
MorseClient morse(&radio);

void setup() {
  
  // set up serial comm
  Serial3.swap(1);
  Serial3.begin(115200);

  // init radio
  SPI.pins(30, 31, 32, 33);
  // initialize RFM69 with default settings
  Serial3.print(F("[RFM69] Initializing ... "));
  int state = radio.begin();

  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    while (true);
  }

  // radio output power
  // only for RFM69HW!
  Serial3.print(F("[RF69] Setting high power module ... "));
  state = radio.setOutputPower(20, true);
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    while (true);
  }

  // initialize Morse client
  Serial3.print(F("[Morse] Initializing ... "));
 
  state = morse.begin(FREQ);
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    while (true);
  }
}

void loop() {
  Serial3.print(F("[Morse] Sending Morse data ... "));

  // MorseClient supports all methods of the Serial class
  // NOTE: Characters that do not have ITU-R M.1677-1
  //       representation will not be sent! Lower case
  //       letters will be capitalized.

  // send start signal first
  morse.startSignal();

  // Arduino String class
  String aStr = "HI FROM OPENSTRATO KIT";
  morse.print(aStr);


  Serial3.println(F("done!"));

  // wait for a second before transmitting again
  delay(1000);
}
