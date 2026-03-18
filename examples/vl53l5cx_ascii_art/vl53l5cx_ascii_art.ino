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
#include <Wire.h>

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;

// --- ASCII density ramps (space = far, last char = close) ---
// Pick one by changing RAMP_SELECT (0-5)
#define RAMP_SELECT 0

#if RAMP_SELECT == 0
// 10-char classic
const char densityRamp[] = " .:-=+*#%@";
#elif RAMP_SELECT == 1
// 24-char fine gradient (Paul Bourke subset)
const char densityRamp[] = " .`^,:;!i~+][)(zU#MW&%B@";
#elif RAMP_SELECT == 2
// 16-char hex-density: shows relative distance as hex digit
const char densityRamp[] = " 123456789ABCDEF";
#elif RAMP_SELECT == 3
// 10-char dot-matrix: chunky, high contrast
const char densityRamp[] = " .oO08@#MW";
#elif RAMP_SELECT == 4
// 7-char minimal: for tiny terminals
const char densityRamp[] = " .:+#@";
#elif RAMP_SELECT == 5
// 70-char full Paul Bourke ramp: maximum smoothness
const char densityRamp[] =
    " .'`^\",:;Il!i><~+_-?][}{1)(|/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$";
#endif

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

  if (!vl53l5cx.begin()) {
    Serial.println(F("Failed to initialize VL53L5CX sensor!"));
    while (1)
      delay(10);
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

  // Start ranging
  if (!vl53l5cx.startRanging()) {
    Serial.println(F("Failed to start ranging!"));
    while (1)
      delay(10);
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
      printGrid();

      // Print legend
      Serial.println();
      printLegend();
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

// Print the 8x8 grid with proper indexing
void printGrid() {
  const uint8_t width = 8;

  for (int y = 0; y <= width * (width - 1); y += width) {
    for (int x = width - 1; x >= 0; x--) {
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
      Serial.print(' '); // Space between chars for square aspect ratio
    }
    Serial.println();
  }
}

// Print a dynamic legend showing the ramp and distance range
void printLegend() {
  Serial.print(F("Ramp: ["));
  for (int i = rampLength - 1; i >= 0; i--) {
    Serial.print(densityRamp[i]);
  }
  Serial.print(F("] close<"));
  Serial.print(minDist);
  Serial.print(F("mm ... "));
  Serial.print(maxDist);
  Serial.println(F("mm>far  [?]=invalid"));
}
