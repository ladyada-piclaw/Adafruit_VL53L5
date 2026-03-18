/*!
 * @file hw_test_06_target_order.ino
 *
 * Hardware test: VL53L5CX target order set/get
 *
 * Tests:
 *  1. Default target order readback
 *  2. Set CLOSEST (1), readback matches
 *  3. Set STRONGEST (2), readback matches
 *  4. Switch back to CLOSEST, readback matches
 *  5. Range with CLOSEST order, data valid
 *  6. Range with STRONGEST order, data valid
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>

#include "hw_test_helper.h"

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 06: Target Order ==="));
  Serial.println();

  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE, 1000000)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  vl53l5cx.setResolution(16);
  vl53l5cx.setRangingFrequency(15);

  // Test 1: Default target order
  uint8_t order = vl53l5cx.getTargetOrder();
  Serial.print(F("   Default order: "));
  Serial.println(order);
  report("1. Default target order readable",
         order == VL53L5CX_TARGET_ORDER_CLOSEST ||
             order == VL53L5CX_TARGET_ORDER_STRONGEST);

  // Test 2: Set CLOSEST
  bool setC = vl53l5cx.setTargetOrder(VL53L5CX_TARGET_ORDER_CLOSEST);
  order = vl53l5cx.getTargetOrder();
  Serial.print(F("   Set CLOSEST, readback: "));
  Serial.println(order);
  report("2. Set/get CLOSEST (1)",
         setC && order == VL53L5CX_TARGET_ORDER_CLOSEST);

  // Test 3: Set STRONGEST
  bool setS = vl53l5cx.setTargetOrder(VL53L5CX_TARGET_ORDER_STRONGEST);
  order = vl53l5cx.getTargetOrder();
  Serial.print(F("   Set STRONGEST, readback: "));
  Serial.println(order);
  report("3. Set/get STRONGEST (2)",
         setS && order == VL53L5CX_TARGET_ORDER_STRONGEST);

  // Test 4: Switch back to CLOSEST
  bool setBack = vl53l5cx.setTargetOrder(VL53L5CX_TARGET_ORDER_CLOSEST);
  order = vl53l5cx.getTargetOrder();
  Serial.print(F("   Back to CLOSEST, readback: "));
  Serial.println(order);
  report("4. Switch back to CLOSEST",
         setBack && order == VL53L5CX_TARGET_ORDER_CLOSEST);

  VL53L5CX_ResultsData results;

  // Test 5: Range with CLOSEST
  vl53l5cx.setTargetOrder(VL53L5CX_TARGET_ORDER_CLOSEST);
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard first
  bool readC = waitAndRead(&results);
  vl53l5cx.stopRanging();
  uint8_t validC = 0;
  if (readC) {
    for (uint8_t i = 0; i < 16; i++) {
      if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000)
        validC++;
    }
    Serial.print(F("   CLOSEST valid zones: "));
    Serial.print(validC);
    Serial.println(F("/16"));
  }
  report("5. Range with CLOSEST order", readC && validC > 0);

  // Test 6: Range with STRONGEST
  vl53l5cx.setTargetOrder(VL53L5CX_TARGET_ORDER_STRONGEST);
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard first
  bool readS = waitAndRead(&results);
  vl53l5cx.stopRanging();
  uint8_t validS = 0;
  if (readS) {
    for (uint8_t i = 0; i < 16; i++) {
      if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000)
        validS++;
    }
    Serial.print(F("   STRONGEST valid zones: "));
    Serial.print(validS);
    Serial.println(F("/16"));
  }
  report("6. Range with STRONGEST order", readS && validS > 0);

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
    delay(1);
  }
  return false;
}
