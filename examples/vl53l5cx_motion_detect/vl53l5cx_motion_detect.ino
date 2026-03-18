/*!
 * @file vl53l5cx_motion_detect.ino
 *
 * Motion detection example for Adafruit VL53L5CX 8x8 ToF sensor
 *
 * Demonstrates the motion indicator feature which detects movement
 * within the sensor's field of view. The 8x8 grid is divided into
 * 32 motion aggregates (each covering 2 zones).
 *
 * Uses ANSI escape codes for smooth terminal animation.
 *
 * Connect the sensor via STEMMA QT/I2C.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_VL53L5CX.h>
#include <Wire.h>

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(1000000);
  Serial.println(F("Adafruit VL53L5CX Motion Detection Demo"));
  Serial.println(F("========================================"));
  Serial.println(F("Initializing sensor... (this can take up to 10 seconds)"));

  if (!vl53l5cx.begin()) {
    Serial.println(F("Failed to initialize VL53L5CX sensor!"));
    while (1) delay(10);
  }

  Serial.println(F("Sensor initialized!"));

  // Set 8x8 resolution (64 zones)
  if (!vl53l5cx.setResolution(64)) {
    Serial.println(F("Failed to set resolution!"));
  }

  // Set ranging frequency to 15 Hz
  if (!vl53l5cx.setRangingFrequency(15)) {
    Serial.println(F("Failed to set ranging frequency!"));
  }

  // Stop ranging before initializing motion indicator
  vl53l5cx.stopRanging();

  // Initialize motion indicator with 8x8 resolution (64)
  // This creates 32 motion aggregates (each covers 2 zones)
  if (!vl53l5cx.initMotionIndicator(64)) {
    Serial.println(F("Failed to init motion indicator!"));
  }

  // Set motion detection distance range: 400mm to 1500mm
  // Motion is only detected for objects within this range
  if (!vl53l5cx.setMotionDistance(400, 1500)) {
    Serial.println(F("Failed to set motion distance!"));
  }

  // Start ranging
  if (!vl53l5cx.startRanging()) {
    Serial.println(F("Failed to start ranging!"));
    while (1) delay(10);
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

      Serial.println(F("VL53L5CX Motion Detection"));
      Serial.println(F("=========================\n"));

      uint8_t aggregates = results.motion_indicator.nb_of_detected_aggregates;

      if (aggregates > 0) {
        Serial.print(F(">>> MOTION DETECTED! ("));
        Serial.print(aggregates);
        Serial.println(F(" zones) <<<\n"));
      } else {
        Serial.println(F("    No motion              \n"));
      }

      // Print motion grid (4 rows x 8 columns = 32 aggregates)
      Serial.println(F("Motion Grid (32 aggregates):"));
      printMotionGrid();

      // Print legend
      Serial.println();
      Serial.println(F("Legend: [0]=no motion, [>0]=motion detected"));
      Serial.println(F("Range: 400mm to 1500mm"));
    }
  }

  delay(5);  // Small delay between polling
}

// Print the 4x8 motion grid
void printMotionGrid() {
  for (uint8_t row = 0; row < 4; row++) {
    for (uint8_t col = 0; col < 8; col++) {
      uint8_t idx = row * 8 + col;
      uint32_t motion = results.motion_indicator.motion[idx];

      // Print with fixed width for alignment
      if (motion > 0) {
        Serial.print(F("["));
        printPadded(motion, 4);
        Serial.print(F("]"));
      } else {
        Serial.print(F("[  . ]"));
      }
      Serial.print(F(" "));
    }
    Serial.println();
  }
}

// Print a number with padding
void printPadded(uint32_t value, uint8_t width) {
  // Calculate digits
  uint32_t temp = value;
  uint8_t digits = 1;
  while (temp >= 10) {
    temp /= 10;
    digits++;
  }

  // Print leading spaces
  for (uint8_t i = digits; i < width; i++) {
    Serial.print(F(" "));
  }
  Serial.print(value);
}
