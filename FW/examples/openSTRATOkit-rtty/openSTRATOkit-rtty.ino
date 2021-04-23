/*
   OpenSTRATOkit RTTY Transmit Example


   @DevOK1RAJ Team, code by OK1CDJ 12/2020 
    
   This example sends RTTY message using RFM69W
   FSK modem and RadioLib.


   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/

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
#include <RadioLib.h>

// Radio Settings

#define FREQ 434.000
#define SHIFT 610
#define BAUD 300
#define ENC ASCII
#define STOPB 2

// RFM69W  has the following connections:
// NSS pin:   33
// DIO0 pin:  9
// RESET pin: RADIOLIB_NC

RF69 radio = new Module(33, 9, RADIOLIB_NC);

// create RTTY client instance using the FSK module
RTTYClient rtty(&radio);

void setup() {
  Serial3.swap(1);
  Serial3.begin(9600);

  // initialize RFM69 with default settings
  Serial3.print(F("[RF69] Initializing ... "));
  SPI.pins(30, 31, 32, 33);
  int state = radio.begin();

  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    while (true);
  }


  Serial3.print(F("[RTTY] Initializing ... "));

  state = rtty.begin(FREQ, SHIFT, BAUD, ENC, STOPB);
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    while (true);
  }


}

void loop() {
  Serial3.print(F("[RTTY] Sending RTTY data ... "));

  // send out idle condition for 500 ms
  rtty.idle();
  delay(1000);

  // RTTYClient supports all methods of the Serial class

  // Arduino String class
  String aStr = "RYRYRYRYRYRYRYRY  OpenSTRATOkit RTTY Transmit Example RYRYRYRYRYRYRYRY";
  rtty.println(aStr);

  Serial3.println(F("done!"));

  // wait for a second before transmitting again
  delay(1000);
}
