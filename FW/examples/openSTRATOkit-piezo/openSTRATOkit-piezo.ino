/*
   openSTRATOkit piezo example


   @DevOK1RAJ Team, code by oktkas 5/2021
  

 */

#define PIEZO 8

void setup() {
  pinMode(PIEZO, OUTPUT);
  digitalWrite(PIEZO, LOW); //turn the speaker off
}

void loop() {

  // make 2 quick beeps
  digitalWrite(PIEZO, HIGH);
  delay(100);
  digitalWrite(PIEZO, LOW);
  delay(100);
  digitalWrite(PIEZO, HIGH);
  delay(100);
  digitalWrite(PIEZO, LOW);

  delay(1000); // wait 1 second
  
}
