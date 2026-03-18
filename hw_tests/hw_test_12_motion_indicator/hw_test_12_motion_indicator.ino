/*!
 * @file hw_test_12_motion_indicator.ino
 *
 * Hardware test: VL53L5CX motion indicator feature
 *
 * Tests:
 *  1. initMotionIndicator(64) returns true
 *  2. setMotionDistance(400, 1500) returns true
 *  3. Collect 20 frames and print motion data
 *  4. setMotionResolution(16) returns true
 *  5. setMotionResolution(64) returns true (switch back)
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_VL53L5CX.h>
#include "hw_test_helper.h"

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("=== HW Test 12: Motion Indicator ==="));
  Serial.println();

  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE)) {
    Serial.println(F("Init failed!"));
    while (1) delay(10);
  }

  // Set 8x8 resolution, 15Hz
  vl53l5cx.setResolution(64);
  vl53l5cx.setRangingFrequency(15);

  // Test 1: initMotionIndicator
  Serial.println(F("   Stopping ranging for motion init..."));
  vl53l5cx.stopRanging();

  bool initOk = vl53l5cx.initMotionIndicator(64);
  Serial.print(F("   initMotionIndicator(64): "));
  Serial.println(initOk ? F("OK") : F("FAIL"));
  report("1. initMotionIndicator(64)", initOk);

  // Test 2: setMotionDistance
  bool distOk = vl53l5cx.setMotionDistance(400, 1500);
  Serial.print(F("   setMotionDistance(400, 1500): "));
  Serial.println(distOk ? F("OK") : F("FAIL"));
  report("2. setMotionDistance(400, 1500)", distOk);

  // Start ranging for frame collection
  vl53l5cx.startRanging();

  // Test 3: Collect 20 frames
  Serial.println();
  Serial.println(F("   Collecting 20 frames..."));
  Serial.println();

  uint8_t framesCollected = 0;
  uint8_t framesWithMotion = 0;
  unsigned long timeout = millis() + 10000;  // 10 second timeout

  while (framesCollected < 20 && millis() < timeout) {
    if (vl53l5cx.isDataReady()) {
      if (vl53l5cx.getRangingData(&results)) {
        framesCollected++;

        uint8_t aggregates = results.motion_indicator.nb_of_detected_aggregates;
        if (aggregates > 0) {
          framesWithMotion++;
        }

        Serial.print(F("   Frame "));
        Serial.print(framesCollected);
        Serial.print(F(": aggregates="));
        Serial.print(aggregates);
        Serial.print(F(" motion=["));

        printMotionArray(32);

        Serial.println(F("]"));
      }
    }
    delay(5);
  }

  Serial.println();
  Serial.print(F("   Frames collected: "));
  Serial.print(framesCollected);
  Serial.print(F("/20, frames with motion: "));
  Serial.println(framesWithMotion);

  // Pass if we got all 20 frames (motion detection depends on actual movement)
  report("3. Collect 20 frames", framesCollected == 20);

  // Test 4: setMotionResolution(16) for 4x4 mode
  vl53l5cx.stopRanging();
  bool res16Ok = vl53l5cx.setMotionResolution(16);
  Serial.print(F("   setMotionResolution(16): "));
  Serial.println(res16Ok ? F("OK") : F("FAIL"));
  report("4. setMotionResolution(16)", res16Ok);

  // Test 5: setMotionResolution(64) back to 8x8
  bool res64Ok = vl53l5cx.setMotionResolution(64);
  Serial.print(F("   setMotionResolution(64): "));
  Serial.println(res64Ok ? F("OK") : F("FAIL"));
  report("5. setMotionResolution(64)", res64Ok);

  // Summary
  Serial.println();
  Serial.println(F("=== SUMMARY ==="));
  Serial.print(passed);
  Serial.print(F(" passed, "));
  Serial.print(failed);
  Serial.println(F(" failed"));
  Serial.println();

  if (failed == 0) {
    Serial.println(F("PASS: hw_test_12"));
  } else {
    Serial.println(F("FAIL: hw_test_12"));
  }
}

void loop() {
  delay(1000);
}

void report(const char *name, bool ok) {
  Serial.print(name);
  if (ok) {
    Serial.println(F(" ... PASSED"));
    passed++;
  } else {
    Serial.println(F(" ... FAILED"));
    failed++;
  }
}

void printMotionArray(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    if (i > 0) {
      Serial.print(F(","));
    }
    Serial.print(results.motion_indicator.motion[i]);
  }
}
