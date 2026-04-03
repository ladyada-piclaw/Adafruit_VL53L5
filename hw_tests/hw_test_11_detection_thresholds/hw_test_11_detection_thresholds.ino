/*!
 * @file hw_test_11_detection_thresholds.ino
 *
 * Hardware test: VL53L5CX Detection Thresholds
 *
 * Tests the detection threshold API:
 *  1. Set thresholds via setDetectionThresholds()
 *  2. Enable thresholds via setDetectionThresholdsEnable()
 *  3. Read back enabled state and verify
 *  4. Read back threshold values and verify
 *  5. Disable thresholds and verify
 *  6. Range with thresholds enabled (data still flows)
 *
 * NOTE: INT pin behavior with thresholds enabled is a known limitation.
 * The VL53L5CX stops asserting INT for data-ready when thresholds are
 * active, and threshold-triggered INT has not been observed reliably.
 * This test validates the API only, not INT+threshold correlation.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_VL53L5CX.h>

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;
VL53L5CX_DetectionThresholds thresholds[VL53L5CX_NB_THRESHOLDS];

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 11: Detection Thresholds ==="));
  Serial.println();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &Wire, 400000)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  // Set 4x4 resolution, 15Hz
  vl53l5cx.setResolution(16);
  vl53l5cx.setRangingFrequency(15);

  // Must stop ranging before configuring thresholds
  vl53l5cx.stopRanging();

  // Initialize all thresholds to zero
  memset(thresholds, 0, sizeof(thresholds));

  // Configure zone 0: trigger when distance <= 500mm
  thresholds[0].zone_num = 0;
  thresholds[0].measurement = VL53L5CX_DISTANCE_MM;
  thresholds[0].type = VL53L5CX_LESS_THAN_EQUAL_MIN_CHECKER;
  thresholds[0].param_low_thresh = 500;
  thresholds[0].param_high_thresh = 500;
  thresholds[0].mathematic_operation = VL53L5CX_OPERATION_OR;

  // Terminate the threshold list
  thresholds[1].zone_num = VL53L5CX_LAST_THRESHOLD;

  // Test 1: Set detection thresholds
  bool setOk = vl53l5cx.setDetectionThresholds(thresholds);
  report("1. setDetectionThresholds()", setOk);

  // Test 2: Enable detection thresholds
  bool enableOk = vl53l5cx.setDetectionThresholdsEnable(true);
  report("2. setDetectionThresholdsEnable(true)", enableOk);

  // Test 3: Read back enabled state
  bool isEnabled = vl53l5cx.getDetectionThresholdsEnable();
  report("3. getDetectionThresholdsEnable() == true", isEnabled);

  // Test 4: Read back thresholds and verify zone 0 values
  VL53L5CX_DetectionThresholds readBack[VL53L5CX_NB_THRESHOLDS];
  bool getOk = vl53l5cx.getDetectionThresholds(readBack);
  Serial.print(F("   Read back: zone_num="));
  Serial.print(readBack[0].zone_num);
  Serial.print(F(", measurement="));
  Serial.print(readBack[0].measurement);
  Serial.print(F(", type="));
  Serial.print(readBack[0].type);
  Serial.print(F(", low="));
  Serial.print(readBack[0].param_low_thresh);
  Serial.print(F(", high="));
  Serial.println(readBack[0].param_high_thresh);

  bool valuesMatch =
      getOk && (readBack[0].zone_num == 0) &&
      (readBack[0].measurement == VL53L5CX_DISTANCE_MM) &&
      (readBack[0].type == VL53L5CX_LESS_THAN_EQUAL_MIN_CHECKER) &&
      (readBack[0].param_low_thresh == 500);
  report("4. getDetectionThresholds() values match", valuesMatch);

  // Test 5: Disable thresholds and verify
  bool disableOk = vl53l5cx.setDetectionThresholdsEnable(false);
  bool isDisabled = !vl53l5cx.getDetectionThresholdsEnable();
  report("5. Disable thresholds round-trip", disableOk && isDisabled);

  // Re-enable for ranging test
  vl53l5cx.setDetectionThresholdsEnable(true);

  // Test 6: Start ranging and verify data still flows with thresholds enabled
  vl53l5cx.startRanging();

  Serial.println(F("   Waiting for data with thresholds enabled..."));
  bool gotData = false;
  unsigned long start = millis();
  while (millis() - start < 10000) {
    if (vl53l5cx.isDataReady()) {
      vl53l5cx.getRangingData(&results);
      uint8_t validZones = 0;
      for (uint8_t i = 0; i < 16; i++) {
        if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000) {
          validZones++;
        }
      }
      Serial.print(F("   Valid zones: "));
      Serial.print(validZones);
      Serial.println(F("/16"));
      gotData = (validZones > 0);
      break;
    }
    delay(10);
  }
  report("6. Range with thresholds enabled", gotData);

  // Summary
  printSummary();
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

void printSummary() {
  Serial.println();
  Serial.println(F("=== SUMMARY ==="));
  Serial.print(passed);
  Serial.print(F(" passed, "));
  Serial.print(failed);
  Serial.println(F(" failed"));
  Serial.println();
  if (failed == 0) {
    Serial.println(F("PASS: hw_test_11"));
  } else {
    Serial.println(F("FAIL: hw_test_11"));
  }
}
