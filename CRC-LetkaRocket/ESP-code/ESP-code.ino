// some libraries for ESP32 cam:
#include "Arduino.h"
//#include "esp_camera.h"

// libraries for mpu and bme sensor:
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// i2c and math library
#include <Wire.h>
#include <Math.h>

//define pin numbers:
#define BUZZER_PIN 13
#define LOCK_PIN 12
#define BUTTON 11

//define some usefull values:
#define MAX_ACC 13
#define MIN_ACC 5
#define TIME_AFTER_START 30000

// -----------------I2C-----------------
// this needs to be defined because of esp
#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15
TwoWire I2CSensors = TwoWire(0);

// some structure for mpu sensor
Adafruit_MPU6050 mpu;

// structure for bme sensor
/*
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
*/

// create variables for time measurment
int start = 0;
int wasLess = 0;
int timeOver = 0;

void setup(void) {
  // pin 12 and 13 will serve to throw the parachute from rocket
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LOCK_PIN, OUTPUT);
  pinMode(LOCK_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  
  // esp don't need to wait
  //while (!Serial)
  //delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Letka GML rocket");

  // open the lock of the rocket parachute
  digitalWrite(LOCK_PIN, HIGH)

  // initialize i2c pins - necessary for ESP
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);

  // Try to initialize! For ESP we need to specify address and which I2C pins we will use
  if (!mpu.begin(0x68, &I2CSensors)) {
    Serial.println("Failed to find MPU6050 chip");
  }

  if (!bme.begin(0x76, &I2CSensors)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
  }
  
  Serial.println("MPU6050 Found!");

  // this code will read and set ranges - unnecessary:
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // wait for press the button:
  while(!digitalRead(BUTTON_PIN)){
    delay(10);
  }
  
  // lock the rocket:
  digitalWrite(pinMode(LOCK_PIN, LOW))
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

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
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
  Serial.print(pressure_event.pressure);
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
  if(millis()-start>TIME_AFTER_START && !timeOver){
    timeOver = 1;
    digitalWrite(LOCK_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("Parachute opened!");
  }
  // for final program **COMMENT THIS** delay:
  delay(100);
}
