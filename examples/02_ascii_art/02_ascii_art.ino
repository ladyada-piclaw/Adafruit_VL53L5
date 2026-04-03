/*!
 * @file vl53l5cx_ascii_art.ino
 *
 * ASCII art visualization for Adafruit VL53L5CX 8x8 ToF sensor
 *
 * Displays a live ASCII art animation of the distance grid on the serial
 * terminal. Uses ANSI escape codes to animate in place.
 *
 * Connect the sensor via STEMMA QT/I2C.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_VL53L5CX.h>

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;

// 70-char Paul Bourke density ramp: maximum smoothness
const char densityRamp[] =
    " .'`^\",:;Il!i><~+_-?][}{1)(|/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$";

const int rampLength = sizeof(densityRamp) - 1; // auto-calculated
const int minDist = 200;  // mm - closest (maps to last char)
const int maxDist = 2000; // mm - farthest (maps to ' ')

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("Adafruit VL53L5CX ASCII Art Demo"));
  Serial.println(F("================================="));
  Serial.println(F("Initializing sensor... (this can take up to 10 seconds)"));

  // Initialize with I2C address 0x29, Wire bus, 400kHz clock
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

  Serial.println(F("Starting ASCII art display...\n"));
  delay(500);

  // Clear screen and hide cursor
  Serial.print(F("\033[2J\033[?25l"));
}

void loop() {
  if (vl53l5cx.isDataReady()) {
    if (vl53l5cx.getRangingData(&results)) {
      // Cursor home - animate in place
      Serial.print("\033[H");

      Serial.println(F("VL53L5CX 8x8 Distance Grid"));
      Serial.println(F("==========================\n"));

      // Print the 8x8 grid
      const uint8_t width = 8;
      for (int x = width - 1; x >= 0; x--) {
        for (int y = width * (width - 1); y >= 0; y -= width) {
          int idx = x + y;
          uint8_t status = results.target_status[idx];

          char c;
          // Status 5 = valid, Status 9 = valid but sigma high
          if (status == 5 || status == 9) {
            c = distanceToChar(results.distance_mm[idx]);
          } else {
            c = '?'; // Invalid zone
          }

          Serial.print(c);
          Serial.print(' ');
        }
        Serial.println();
      }

      Serial.println();
    }
  }

  delay(5); // Small delay between polling
}

// Convert distance to ASCII character
char distanceToChar(int16_t distance_mm) {
  if (distance_mm <= minDist) {
    return densityRamp[rampLength - 1]; // '@' for very close
  }
  if (distance_mm >= maxDist) {
    return densityRamp[0]; // ' ' for very far
  }

  // Linear mapping: closer = higher index (denser character)
  int idx = (int)((long)(maxDist - distance_mm) * (rampLength - 1) /
                  (maxDist - minDist));
  return densityRamp[idx];
}

void halt(const __FlashStringHelper* msg) {
  Serial.println(msg);
  while (1)
    delay(10);
}
