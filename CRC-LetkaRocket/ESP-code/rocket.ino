// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;
int start = 0;
int wasLess = 0;
int timeOver = 0;

void setup(void) {
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

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
  float total = 0.0;
  /* Print out the values */
  total += square(a.acceleration.x);
  total += square(a.acceleration.y);
  total += square(a.acceleration.z);
  
  // compute totl value of acceleration (in all axes)
  total = sqrt(total);
  Serial.println(total);
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
