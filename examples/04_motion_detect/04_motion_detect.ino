/*!
 * @file 04_motion_detect.ino
 *
 * Motion detection example for Adafruit VL53L5CX ToF sensor
 *
 * Demonstrates the motion indicator feature which detects movement
 * within the sensor's field of view. Uses 4x4 resolution which gives
 * 16 motion aggregates with clean 1:1 zone mapping.
 *
 * Uses ANSI escape codes for smooth terminal animation.
 *
 * Connect the sensor via STEMMA QT/I2C.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_VL53L5CX.h>

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("Adafruit VL53L5CX Motion Detection Demo"));
  Serial.println(F("========================================"));
  Serial.println(F("Initializing sensor... (this can take up to 10 seconds)"));

  if (!vl53l5cx.begin()) {
    Serial.println(F("Failed to initialize VL53L5CX sensor!"));
    while (1)
      delay(10);
  }

  Serial.println(F("Sensor initialized!"));

  // Use 4x4 resolution (16 zones = 16 motion aggregates)
  if (!vl53l5cx.setResolution(16)) {
    Serial.println(F("Failed to set resolution!"));
  }

  // Set ranging frequency to 15 Hz
  if (!vl53l5cx.setRangingFrequency(15)) {
    Serial.println(F("Failed to set ranging frequency!"));
  }

  // Stop ranging before initializing motion indicator
  vl53l5cx.stopRanging();

  // Initialize motion indicator with 4x4 resolution (16)
  if (!vl53l5cx.initMotionIndicator(16)) {
    Serial.println(F("Failed to init motion indicator!"));
  }

  // Set motion detection distance range: 400mm to 1500mm
  if (!vl53l5cx.setMotionDistance(400, 1500)) {
    Serial.println(F("Failed to set motion distance!"));
  }

  // Start ranging
  if (!vl53l5cx.startRanging()) {
    Serial.println(F("Failed to start ranging!"));
    while (1)
      delay(10);
  }

  Serial.println(F("Starting motion detection...\n"));
  delay(500);

  // Clear screen and hide cursor
  Serial.print(F("\033[2J\033[?25l"));
}

void loop() {
  if (vl53l5cx.isDataReady()) {
    if (vl53l5cx.getRangingData(&results)) {
      // Cursor home - animate in place
      Serial.print(F("\033[H"));

      Serial.println(F("VL53L5CX Motion Detection (4x4)"));
      Serial.println(F("===============================\n"));

      uint8_t aggregates = results.motion_indicator.nb_of_detected_aggregates;

      if (aggregates > 0) {
        Serial.print(F(">>> MOTION DETECTED! ("));
        Serial.print(aggregates);
        Serial.println(F(" zones) <<<\n"));
      } else {
        Serial.println(F("    No motion              \n"));
      }

      // Print 4x4 motion grid
      for (uint8_t row = 0; row < 4; row++) {
        for (uint8_t col = 0; col < 4; col++) {
          uint8_t idx = row * 4 + col;
          uint32_t motion = results.motion_indicator.motion[idx];

          Serial.print(F("["));
          if (motion > 0) {
            // Print with padding for alignment
            if (motion < 10)
              Serial.print(F("   "));
            else if (motion < 100)
              Serial.print(F("  "));
            else if (motion < 1000)
              Serial.print(F(" "));
            Serial.print(motion);
          } else {
            Serial.print(F("   0"));
          }
          Serial.print(F("] "));
        }
        Serial.println();
      }

      Serial.println();
      Serial.println(F("Range: 400mm to 1500mm         "));
    }
  }

  delay(5);
}
