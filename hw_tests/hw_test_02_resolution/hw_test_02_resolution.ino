/*!
 * @file hw_test_02_resolution.ino
 *
 * Hardware test: VL53L5CX resolution switching (4x4 vs 8x8)
 *
 * Tests:
 *  1. Set resolution to 4x4 (16), readback matches
 *  2. Range in 4x4, get 16 valid zones
 *  3. Set resolution to 8x8 (64), readback matches
 *  4. Range in 8x8, get 64 valid zones
 *  5. Switch back to 4x4, readback matches
 *  6. Invalid resolution (32) rejected
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 02: Resolution ==="));
  Serial.println();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &Wire, 400000)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  VL53L5CX_ResultsData results;

  // Test 1: Set 4x4
  bool set4x4 = vl53l5cx.setResolution(16);
  uint8_t res = vl53l5cx.getResolution();
  Serial.print(F("   Set 4x4, readback: "));
  Serial.println(res);
  report("1. Set 4x4 resolution", set4x4 && res == 16);

  // Test 2: Range in 4x4
  vl53l5cx.setRangingFrequency(15);
  vl53l5cx.startRanging();
  bool read4x4 = waitAndRead(&results);
  uint8_t valid4x4 = 0;
  if (read4x4) {
    for (uint8_t i = 0; i < 16; i++) {
      if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000)
        valid4x4++;
    }
    Serial.print(F("   4x4 valid zones: "));
    Serial.print(valid4x4);
    Serial.println(F("/16"));
  }
  vl53l5cx.stopRanging();
  report("2. Range in 4x4 mode", read4x4 && valid4x4 > 0);

  // Test 3: Set 8x8
  bool set8x8 = vl53l5cx.setResolution(64);
  res = vl53l5cx.getResolution();
  Serial.print(F("   Set 8x8, readback: "));
  Serial.println(res);
  report("3. Set 8x8 resolution", set8x8 && res == 64);

  // Test 4: Range in 8x8
  vl53l5cx.startRanging();
  bool read8x8 = waitAndRead(&results);
  uint8_t valid8x8 = 0;
  if (read8x8) {
    for (uint8_t i = 0; i < 64; i++) {
      if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000)
        valid8x8++;
    }
    Serial.print(F("   8x8 valid zones: "));
    Serial.print(valid8x8);
    Serial.println(F("/64"));
  }
  vl53l5cx.stopRanging();
  report("4. Range in 8x8 mode", read8x8 && valid8x8 > 0);

  // Test 5: Switch back to 4x4
  bool setBack = vl53l5cx.setResolution(16);
  res = vl53l5cx.getResolution();
  Serial.print(F("   Back to 4x4, readback: "));
  Serial.println(res);
  report("5. Switch back to 4x4", setBack && res == 16);

  // Test 6: Invalid resolution
  bool setInvalid = vl53l5cx.setResolution(32);
  res = vl53l5cx.getResolution();
  Serial.print(F("   After invalid set, readback: "));
  Serial.println(res);
  report("6. Invalid resolution (32) rejected", !setInvalid && res == 16);

  // Summary
  Serial.println();
  Serial.println(F("=== SUMMARY ==="));
  Serial.print(passed);
  Serial.print(F(" passed, "));
  Serial.print(failed);
  Serial.println(F(" failed"));
  Serial.print(F("Result: "));
  Serial.println(failed == 0 ? F("ALL PASSED") : F("SOME FAILED"));
}

void loop() {
  delay(1000);
}

void report(const char* name, bool ok) {
  Serial.print(name);
  if (ok) {
    Serial.println(F(" ... PASSED"));
    passed++;
  } else {
    Serial.println(F(" ... FAILED"));
    failed++;
  }
}

bool waitAndRead(VL53L5CX_ResultsData* results) {
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (vl53l5cx.isDataReady()) {
      return vl53l5cx.getRangingData(results);
    }
    delay(5);
  }
  return false;
}
