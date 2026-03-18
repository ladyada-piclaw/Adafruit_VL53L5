/*!
 * @file vl53l5cx_simpletest.ino
 *
 * Simple test for Adafruit VL53L5CX 8x8 ToF sensor
 *
 * Reads an 8x8 array of distances and prints them to Serial.
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
  while (!Serial)
    delay(10);

  Serial.println("Adafruit VL53L5CX simpletest");
  Serial.println("============================");

  // Or use 400000 for 400kHz if 1MHz is unstable

  Serial.println("Initializing sensor... (this can take up to 10 seconds)");

  if (!vl53l5cx.begin()) {
    Serial.println("Failed to initialize VL53L5CX sensor!");
    while (1)
      delay(10);
  }

  Serial.println("Sensor initialized!");

  // Set 8x8 resolution (64 zones)
  if (!vl53l5cx.setResolution(64)) {
    Serial.println("Failed to set resolution!");
  }

  // Set ranging frequency to 15 Hz
  if (!vl53l5cx.setRangingFrequency(15)) {
    Serial.println("Failed to set ranging frequency!");
  }

  // Start ranging
  if (!vl53l5cx.startRanging()) {
    Serial.println("Failed to start ranging!");
    while (1)
      delay(10);
  }

  Serial.print("Resolution: ");
  Serial.println(vl53l5cx.getResolution());
  Serial.print("Ranging frequency: ");
  Serial.print(vl53l5cx.getRangingFrequency());
  Serial.println(" Hz");
  Serial.println();
}

void loop() {
  if (vl53l5cx.isDataReady()) {
    if (vl53l5cx.getRangingData(&results)) {
      // Get resolution to determine grid size
      uint8_t resolution = vl53l5cx.getResolution();
      uint8_t width = (resolution == 16) ? 4 : 8;

      // Print distance array
      // The ST library returns data transposed from zone mapping in datasheet
      // Print with increasing y, decreasing x to reflect reality
      for (int y = 0; y <= width * (width - 1); y += width) {
        for (int x = width - 1; x >= 0; x--) {
          int idx = x + y;
          Serial.print("\t");
          Serial.print(results.distance_mm[idx]);
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  delay(5); // Small delay between polling
}
