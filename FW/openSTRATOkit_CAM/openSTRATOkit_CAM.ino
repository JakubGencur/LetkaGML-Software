/*
  openSTRATOkit CAM FW

  @DevOK1RAJ Team, code by OK1CDJ 12/2020

  Takes 10 pictures every 90 seconds (DELAY 90) and saves to SD card

  Place reset.txt into the root of the SD card to reset file numbering

  Compile and upload instructions:
  - In your Arduino IDE, go to File > Preferences and enter
    https://dl.espressif.com/dl/package_esp32_index.json
    into the “Additional Board Manager URLs” field
  - Go to Tools > Board > Boards Manager…, search for ESP32 and install
    ESP32 by Espressif Systems
  - In the Tools > Board menu select "AI Thinker ESP32-CAM"
  - GPIO 0 must be connected to GND to upload a sketch
  - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button
    to put your board in flashing mode
  - Connect the board via a USB to Serial converter and select its port in Tools > Port
  - Wiring diagram https://www.files.dotknisevesmiru.cz/ESP32CAM_upload.png
*/

// libraries
#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <Preferences.h>
#include "driver/adc.h"
#include <WiFi.h>
#include <esp_bt.h>

// pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds
#define DELAY 90 // delay between series of pictures [s]
#define BATCH_PHOTOS 10 // number of pictures to be taken in one series
#define STATUS_LED 2 // GPIO pin of the STATUS LED (SD CARD DATA PIN!)

long pictureNumber = 0;

Preferences preferences;

void setup() {
  //disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // serial
  Serial.begin(115200);
  Serial.println("openSTRATOkit ESP32CAM start");
  Serial.println("Taking " + String(BATCH_PHOTOS) + " pictures every "
                 + String(DELAY) + " seconds");

  // shut off unnecessary power drain
  adc_power_off();
  WiFi.mode(WIFI_OFF);
  esp_bt_controller_disable();

  // camera settings
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // set picture quality
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // init camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // init & check for SD card
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }

  camera_fb_t * fb = NULL;
  
  // take a series of pictures & save them to the SD card
  for (int i=0; i<BATCH_PHOTOS; i++) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }
  
    // initialize preferences
    preferences.begin("cam", false);
    pictureNumber = preferences.getLong("pictureNumber", 0);
    preferences.end();

    // reset picture numbering if requested
    if (SD_MMC.exists("/reset.txt")) {
      Serial.println("reset.txt exists. Reseting file number.");
      pictureNumber = 0;
      SD_MMC.remove("/reset.txt");
    }
    else {
      Serial.println("reset.txt doesn't exist.");
    }

    // save the picture to the SD card
    pictureNumber++;
    String path = "/picture" + String(pictureNumber) + ".jpg";
    fs::FS &fs = SD_MMC;
    Serial.printf("Picture file name: %s\n", path.c_str());
  
    File file = fs.open(path.c_str(), FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.printf("Saved file to path: %s\n", path.c_str());
      preferences.begin("cam", false);
      preferences.putLong("pictureNumber", pictureNumber);
      preferences.end();
    }
    file.close();
    esp_camera_fb_return(fb);
  }
  
  //blink the status LED for <DELAY>
  pinMode(STATUS_LED, OUTPUT);
  for (int i=0; i<DELAY; i++) {
    digitalWrite(STATUS_LED , LOW);
    delay(500);
    digitalWrite(STATUS_LED , HIGH);
    delay(500);
  }

  //soft restart
  ESP.restart();
  
}

void loop() {
  
}
