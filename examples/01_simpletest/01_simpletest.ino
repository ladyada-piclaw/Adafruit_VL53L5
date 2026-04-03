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

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("Adafruit VL53L5CX simpletest"));
  Serial.println(F("============================"));

  // Initialize with I2C address 0x29, Wire bus, 400kHz clock
  // You can also try 1000000 (1MHz) for faster firmware upload
  Serial.println(F("Initializing sensor... (this can take up to 10 seconds)"));

  if (!vl53l5cx.begin(VL53L5CX_DEFAULT_ADDRESS, &Wire, 400000)) {
    halt(F("Failed to initialize VL53L5CX sensor!"));
  }

  Serial.println(F("Sensor initialized!"));

  // Set 8x8 resolution (64 zones)
  if (!vl53l5cx.setResolution(64)) {
    halt(F("Failed to set resolution!"));
  }

  // Set ranging frequency to 15 Hz
  if (!vl53l5cx.setRangingFrequency(15)) {
    halt(F("Failed to set ranging frequency!"));
  }

  // Start ranging
  if (!vl53l5cx.startRanging()) {
    halt(F("Failed to start ranging!"));
  }

  Serial.print(F("Resolution: "));
  Serial.println(vl53l5cx.getResolution());
  Serial.print(F("Ranging frequency: "));
  Serial.print(vl53l5cx.getRangingFrequency());
  Serial.println(F(" Hz"));
  Serial.println();
}

void loop() {
  if (vl53l5cx.isDataReady()) {
    if (vl53l5cx.getRangingData(&results)) {
      // Get resolution to determine grid size
      uint8_t resolution = vl53l5cx.getResolution();
      uint8_t width = 8;
      if (resolution == 16) {
        width = 4;
      }

      // Print distance array
      // Zone order from ST library: index = col + row*width
      for (int x = width - 1; x >= 0; x--) {
        for (int y = width * (width - 1); y >= 0; y -= width) {
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

void halt(const __FlashStringHelper* msg) {
  Serial.println(msg);
  while (1)
    delay(10);
}
