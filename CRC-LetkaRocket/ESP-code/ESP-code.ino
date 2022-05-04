// some libraries for ESP32 cam:
#include "Arduino.h"
//#include "esp_camera.h"

// libraries for mpu sensor:
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// i2c and math library
#include <Wire.h>
#include <Math.h>

// -----------------I2C-----------------
// this needs to be defined because of esp
#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15
TwoWire I2CSensors = TwoWire(0);

// some structure for mpu sensor
Adafruit_MPU6050 mpu;

// create variables for time measurment
int start = 0;
int wasLess = 0;
int timeOver = 0;

void setup(void) {
  // pin 12 and 13 will serve to throw the parachute from rocket
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(115200);
  
  // esp don't need to wait
  //while (!Serial)
  //delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Letka GML rocket");

  // initialize i2c pins - necessary for ESP
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);

  // Try to initialize! For ESP we need to specify address and which I2C pins we will use
  if (!mpu.begin(0x68, &I2CSensors)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // this code will read and set ranges - unnecessary:
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
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
  }
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float total = 0.0; //variable for compute abslute value of acceleration
  
  // compute the squares and add them to total
  total += pow(a.acceleration.x, 2);
  total += pow(a.acceleration.y, 2);
  total += pow(a.acceleration.z, 2);
  
  // compute total value of acceleration (in all axes) by formule:
  // a_t = sqrt(a_x^2 + a_y^2 + a_z^2)
  total = sqrt(total);
  Serial.println(total);

  // when the values are under/above the normal (start or fall of the rocket) it start to
  // count off
  if(total>13 && !start){
    start = millis();
    Serial.println("mill---------------");
  }
  if(total<5 && !wasLess){
    wasLess = 1;
    digitalWrite(13, HIGH);
    Serial.println("13-----------------");
  }
  if(millis()-start>30000 && !timeOver){
    timeOver = 1;
    digitalWrite(12, HIGH);
    Serial.println("12-----------------");
  }
  delay(100);
}
