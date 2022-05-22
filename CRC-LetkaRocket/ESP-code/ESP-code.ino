// This is Letka GML code for module to Czech Rocket Challenge:
// Pinout:
/*
 ESP-pins:  modules pins
 IO16       Button+
 IO15       MPU-SCL             BME-SCL
 IO14       MPU-SDA             BME-SDA
 IO13       Buzzer+
 IO12       Servo-data(yellow)
 UOR        free cable or pin
 UOT        free cable or pin
 VCC        free cable or pin, or power supply
 GND1       free cable or pin, or power supply
 GND2       BUZZER-             BUTTON-     Servo-GND(brown)  BME-GND     MPU-GND
 5V         Servo-VCC(red)      BME-VCC     MPU-VCC

 more about connection you can find in attached file on our github
*/



// some libraries for ESP32 cam:
#include "Arduino.h"
//#include "esp_camera.h"

// libraries for mpu and bme sensor:
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// servo motor library
#include <ESP32Servo.h>

// i2c and math library
#include <Wire.h>
#include <Math.h>

//define pin numbers:
#define BUZZER_PIN 13
#define LOCK_PIN 12
#define BUTTON_PIN 16

//define some usefull values:
#define MAX_ACC 20
#define MIN_ACC 5
#define TIME_AFTER_START 30000
#define LOCK_ANGLE 0
#define UNLOCK_ANGLE 90

// -----------------I2C-----------------
// this needs to be defined because of esp
#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15
TwoWire I2CSensors = TwoWire(0);

// some structure for mpu sensor
Adafruit_MPU6050 mpu;

// structure for bme sensor

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();


// structore for servo
Servo servo;

// create variables for time measurment
int start = 0;
int wasLess = 0;
int timeOver = 0;

void setup(void) {
  // pin 12 and 13 will serve to throw the parachute from rocket
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LOCK_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  Serial.begin(115200);
  
  // esp don't need to wait
  //while (!Serial)
  //delay(10); // will pause Zero, Leonardo, etc until serial console open
  
  //code for active buzzer, to control if esp is started and if it work properly
  tone(BUZZER_PIN, 4186, 100);
  delay(100);
  tone(BUZZER_PIN, 4186, 500);
  delay(100);
  tone(BUZZER_PIN, 4186, 100);
  delay(100);

/* code for buzzer
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
 */

  Serial.println("Letka GML rocket");

  // initialize i2c pins - necessary for ESP
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);

  // Try to initialize! For ESP we need to specify address and which I2C pins we will use
  if (!mpu.begin(0x68, &I2CSensors)) {
    Serial.println("Failed to find MPU6050 chip");

    //code for active buzzer
    tone(BUZZER_PIN, 4186, 500);
    delay(100);
    
    /*code for buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    */
  }
  else{
    Serial.println("MPU6050 Found!");

    //code for active buzzer
    tone(BUZZER_PIN, 4186, 100);
    delay(100);
    
    /*code for buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    */
  }

  // bme initialization
  if (!bme.begin(0x76, &I2CSensors)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));

    //code for active buzzer
    tone(BUZZER_PIN, 4186, 500);
    delay(100);
    
    /*code for buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);*/ //PRI ODKOMENTOVANI DOPSAT HVEZDALOMENO
  }
  else{
    Serial.println("BME280 Found!");

    //code for active buzzer
    tone(BUZZER_PIN, 4186, 100);
    delay(100);
    
    /*code for buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);*/
  }

  
  // initialize servo
  if (!servo.attach(LOCK_PIN)) {
    Serial.println(F("Could not find a valid servo, check wiring!"));

    //code for active buzzer
    tone(BUZZER_PIN, 4186, 500);
    delay(100);
    
    /*code for buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    */
  }
  else{
    Serial.println("Servo Found!");

    //code for active buzzer
    tone(BUZZER_PIN, 4186, 100);
    delay(100);
    
    /*code for buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    */
  }

  // unlock rocket
  servo.write(UNLOCK_ANGLE);

  // this code will read and set ranges - unnecessary:
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  //code for active buzzer
  tone(BUZZER_PIN, 4186, 100);
  delay(100);
  tone(BUZZER_PIN, 4186, 500);
  delay(100);
  tone(BUZZER_PIN, 4186, 100);
  delay(100);

  /*code for passive buzzer
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  */
  
  // wait for press the button:
  while(digitalRead(BUTTON_PIN)){
    delay(10);
  }
  tone(BUZZER_PIN, 4186, 100);
  delay(100);
  
  // lock the rocket:
  servo.write(LOCK_ANGLE);
  /*Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }*/
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, t_a, t, p, h;
  mpu.getEvent(&a, &g, &t_a);
  
  bme_temp->getEvent(&t);
  bme_pressure->getEvent(&p);
  bme_humidity->getEvent(&h);
  
  float total = 0.0; //variable for compute abslute value of acceleration
  
  // compute the squares and add them to total
  total += pow(a.acceleration.x, 2);
  total += pow(a.acceleration.y, 2);
  total += pow(a.acceleration.z, 2);
  
  // compute total value of acceleration (in all axes) by formule:
  // a_t = sqrt(a_x^2 + a_y^2 + a_z^2)
  total = sqrt(total);
  
  Serial.print(F("Total acceleration = "));
  Serial.print(total);
  Serial.println("m/s-2");

  Serial.print("\Temperature ");
  Serial.print(t_a.temperature);
  Serial.println(" *C");

  // Display the results (acceleration is measured in m/s^2)
  Serial.print("Accelaration x = ");
  Serial.print(a.acceleration.x);
  Serial.println(" m/s^2 ");
  Serial.print("Accelaration y = ");
  Serial.print(a.acceleration.y);
  Serial.println(" m/s^2 ");
  Serial.print("Accelaration z = ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2 ");

  // Display the results (rotation is measured in rad/s)
  Serial.print("Gyroscope x = ");
  Serial.print(g.gyro.x);
  Serial.println(" radians/s ");
  Serial.print("Gyroscope y = ");
  Serial.print(g.gyro.y);
  Serial.println(" radians/s ");
  Serial.print("Gyroscope z = ");
  Serial.print(g.gyro.z);
  Serial.println(" radians/s ");

  
  Serial.print(F("Temperature = "));
  Serial.print(t.temperature);
  Serial.println(" *C");

  Serial.print(F("Humidity = "));
  Serial.print(h.relative_humidity);
  Serial.println(" %");

  Serial.print(F("Pressure = "));
  Serial.print(p.pressure);
  Serial.println(" hPa");
  
  Serial.println();

  // when the values are under/above the normal (start or fall of the rocket) it start to
  // count off
  if(total>MAX_ACC && !start){
    start = millis();
    Serial.println("Start to cont off");
  }
  // use one of these conditions to open parachute:
  /*if(total<MIN_ACC && !wasLess){
    wasLess = 1;
    digitalWrite(LOCK_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("Parachute opened!");
  }*/
  // when counting off is done it set value timeOver to don't repeat this condition
  // unlock the parashute and start the buzzer
  if(millis()-start>TIME_AFTER_START && !timeOver && start){
    timeOver = 1;
    servo.write(UNLOCK_ANGLE);
    //for active buzzer
    tone(BUZZER_PIN, 4186);
    //for passive buzzer
    //digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("Parachute opened!");
  }
  // for final program **COMMENT THIS** delay:
  delay(100);
}
