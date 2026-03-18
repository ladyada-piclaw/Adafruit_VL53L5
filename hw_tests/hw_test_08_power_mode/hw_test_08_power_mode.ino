/*!
 * @file hw_test_08_power_mode.ino
 *
 * Hardware test: VL53L5CX power mode (sleep/wakeup)
 *
 * Tests:
 *  1. Default power mode is WAKEUP
 *  2. Set SLEEP, readback matches
 *  3. Set WAKEUP, readback matches
 *  4. Range after wakeup still works
 *  5. Sleep/wake cycle: sleep, wake, range, verify data
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>
#include "hw_test_helper.h"

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

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

bool waitAndRead(VL53L5CX_ResultsData *results) {
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (vl53l5cx.isDataReady()) {
      return vl53l5cx.getRangingData(results);
    }
    delay(1);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("=== HW Test 08: Power Mode ==="));
  Serial.println();

  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE)) {
    Serial.println(F("Init failed!"));
    while (1) delay(10);
  }

  vl53l5cx.setResolution(16);
  vl53l5cx.setRangingFrequency(15);

  // Test 1: Default power mode
  uint8_t pm = vl53l5cx.getPowerMode();
  Serial.print(F("   Default power mode: "));
  Serial.println(pm);
  report("1. Default is WAKEUP (1)", pm == VL53L5CX_POWER_MODE_WAKEUP);

  // Test 2: Set SLEEP
  bool setSleep = vl53l5cx.setPowerMode(VL53L5CX_POWER_MODE_SLEEP);
  pm = vl53l5cx.getPowerMode();
  Serial.print(F("   Set SLEEP, readback: "));
  Serial.println(pm);
  report("2. Set/get SLEEP (0)", setSleep && pm == VL53L5CX_POWER_MODE_SLEEP);

  // Test 3: Set WAKEUP
  bool setWake = vl53l5cx.setPowerMode(VL53L5CX_POWER_MODE_WAKEUP);
  pm = vl53l5cx.getPowerMode();
  Serial.print(F("   Set WAKEUP, readback: "));
  Serial.println(pm);
  report("3. Set/get WAKEUP (1)", setWake && pm == VL53L5CX_POWER_MODE_WAKEUP);

  // Test 4: Range after wakeup
  VL53L5CX_ResultsData results;
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard
  bool readOk = waitAndRead(&results);
  uint8_t valid = 0;
  if (readOk) {
    for (uint8_t i = 0; i < 16; i++) {
      if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000)
        valid++;
    }
    Serial.print(F("   Post-wake valid zones: "));
    Serial.print(valid);
    Serial.println(F("/16"));
  }
  vl53l5cx.stopRanging();
  report("4. Range after wakeup works", readOk && valid > 0);

  // Test 5: Full sleep/wake cycle
  Serial.println(F("   Sleep/wake cycle..."));
  bool sleepOk = vl53l5cx.setPowerMode(VL53L5CX_POWER_MODE_SLEEP);
  delay(500); // stay asleep for 500ms
  bool wakeOk = vl53l5cx.setPowerMode(VL53L5CX_POWER_MODE_WAKEUP);
  delay(100); // settling time

  vl53l5cx.startRanging();
  waitAndRead(&results); // discard
  bool readCycle = waitAndRead(&results);
  uint8_t validCycle = 0;
  if (readCycle) {
    for (uint8_t i = 0; i < 16; i++) {
      if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000)
        validCycle++;
    }
    Serial.print(F("   Cycle valid zones: "));
    Serial.print(validCycle);
    Serial.println(F("/16"));
  }
  vl53l5cx.stopRanging();
  report("5. Sleep/wake cycle + range", sleepOk && wakeOk && readCycle && validCycle > 0);

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
