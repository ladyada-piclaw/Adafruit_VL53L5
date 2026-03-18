/*!
 * @file hw_test_13_xtalk_calibration.ino
 *
 * Hardware test: VL53L5CX Xtalk Calibration API
 *
 * Tests the xtalk calibration wrapper methods:
 *  1. Get default xtalk margin (should be 50)
 *  2. Set xtalk margin to 100, read back
 *  3. Restore margin to 50
 *  4. Get default xtalk cal data (776 bytes, non-zero)
 *  5. Set xtalk cal data round-trip (write back same data)
 *  6. Run calibration (no proper target — just verify API doesn't crash)
 *  7. Get cal data after calibration (should differ from default)
 *
 * NOTE: Test 6 runs calibration without a proper target. The calibration
 * will complete but values won't be meaningful. This tests the API path,
 * not calibration accuracy.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_VL53L5CX.h>

#include "hw_test_helper.h"

Adafruit_VL53L5CX vl53l5cx;
uint8_t xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE];
uint8_t xtalk_data_after[VL53L5CX_XTALK_BUFFER_SIZE];

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 13: Xtalk Calibration ==="));
  Serial.println();

  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE, 1000000)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  // Test 1: Get default xtalk margin
  uint32_t margin = 0;
  bool getOk = vl53l5cx.getXtalkMargin(&margin);
  Serial.print(F("   Default margin: "));
  Serial.println(margin);
  report("1. getXtalkMargin() default == 50", getOk && (margin == 50));

  // Test 2: Set margin to 100, read back
  bool setOk = vl53l5cx.setXtalkMargin(100);
  uint32_t margin2 = 0;
  vl53l5cx.getXtalkMargin(&margin2);
  Serial.print(F("   Set 100, readback: "));
  Serial.println(margin2);
  report("2. setXtalkMargin(100) round-trip", setOk && (margin2 == 100));

  // Test 3: Restore margin to 50
  bool restoreOk = vl53l5cx.setXtalkMargin(50);
  uint32_t margin3 = 0;
  vl53l5cx.getXtalkMargin(&margin3);
  report("3. Restore margin to 50", restoreOk && (margin3 == 50));

  // Test 4: Get default xtalk cal data
  bool getCalOk = vl53l5cx.getXtalkCalData(xtalk_data);
  // Check that buffer isn't all zeros
  uint16_t nonZero = 0;
  for (uint16_t i = 0; i < VL53L5CX_XTALK_BUFFER_SIZE; i++) {
    if (xtalk_data[i] != 0)
      nonZero++;
  }
  Serial.print(F("   Cal data: "));
  Serial.print(VL53L5CX_XTALK_BUFFER_SIZE);
  Serial.print(F(" bytes, "));
  Serial.print(nonZero);
  Serial.println(F(" non-zero"));
  report("4. getXtalkCalData() non-zero", getCalOk && (nonZero > 0));

  // Test 5: Set cal data round-trip (write same data back)
  bool setCalOk = vl53l5cx.setXtalkCalData(xtalk_data);
  // Read it back
  uint8_t verify[VL53L5CX_XTALK_BUFFER_SIZE];
  vl53l5cx.getXtalkCalData(verify);
  bool match = (memcmp(xtalk_data, verify, VL53L5CX_XTALK_BUFFER_SIZE) == 0);
  report("5. setXtalkCalData() round-trip matches", setCalOk && match);

  // Test 6: Run calibration (no proper target — API smoke test)
  // Use 3% reflectance, 4 samples, 600mm distance (ST defaults)
  Serial.println(F("   Running xtalk calibration (no target, ~5s)..."));
  vl53l5cx.stopRanging();
  bool calOk = vl53l5cx.calibrateXtalk(3, 4, 600);
  Serial.print(F("   Calibration returned: "));
  Serial.println(calOk ? F("OK") : F("FAIL"));
  report("6. calibrateXtalk() completes", calOk);

  // Test 7: Get cal data after calibration — should differ from default
  bool getAfterOk = vl53l5cx.getXtalkCalData(xtalk_data_after);
  bool differs =
      (memcmp(xtalk_data, xtalk_data_after, VL53L5CX_XTALK_BUFFER_SIZE) != 0);
  Serial.print(F("   Cal data after: "));
  Serial.println(differs ? F("changed") : F("same as before"));
  report("7. Cal data changed after calibration", getAfterOk && differs);

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
    Serial.println(F("PASS: hw_test_13"));
  } else {
    Serial.println(F("FAIL: hw_test_13"));
  }
}
