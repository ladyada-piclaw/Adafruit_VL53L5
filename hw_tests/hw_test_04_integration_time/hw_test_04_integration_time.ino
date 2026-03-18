/*!
 * @file hw_test_04_integration_time.ino
 *
 * Hardware test: VL53L5CX integration time set/get and noise comparison
 *
 * Tests:
 *  1. Set integration time 2ms, readback matches
 *  2. Set integration time 10ms, readback matches
 *  3. Set integration time 50ms, readback matches
 *  4. Set integration time 100ms, readback matches
 *  5. Set integration time 500ms, readback matches
 *  6. Avg sigma at 10ms integration
 *  7. Avg sigma at 100ms integration (should be lower than 10ms)
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>

#include "hw_test_helper.h"

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

// Average sigma across all zones (lower = less noise)

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 04: Integration Time ==="));
  Serial.println();

  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE, 1000000)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  // Use 4x4 for speed
  vl53l5cx.setResolution(16);
  vl53l5cx.setRangingFrequency(1); // slow freq so integration time dominates

  // Test 1: 2ms
  bool set2 = vl53l5cx.setIntegrationTime(2);
  uint32_t it = vl53l5cx.getIntegrationTime();
  Serial.print(F("   Set 2ms, readback: "));
  Serial.print(it);
  Serial.println(F(" ms"));
  report("1. Set/get 2ms", set2 && it == 2);

  // Test 2: 10ms
  bool set10 = vl53l5cx.setIntegrationTime(10);
  it = vl53l5cx.getIntegrationTime();
  Serial.print(F("   Set 10ms, readback: "));
  Serial.print(it);
  Serial.println(F(" ms"));
  report("2. Set/get 10ms", set10 && it == 10);

  // Test 3: 50ms
  bool set50 = vl53l5cx.setIntegrationTime(50);
  it = vl53l5cx.getIntegrationTime();
  Serial.print(F("   Set 50ms, readback: "));
  Serial.print(it);
  Serial.println(F(" ms"));
  report("3. Set/get 50ms", set50 && it == 50);

  // Test 4: 100ms
  bool set100 = vl53l5cx.setIntegrationTime(100);
  it = vl53l5cx.getIntegrationTime();
  Serial.print(F("   Set 100ms, readback: "));
  Serial.print(it);
  Serial.println(F(" ms"));
  report("4. Set/get 100ms", set100 && it == 100);

  // Test 5: 500ms
  bool set500 = vl53l5cx.setIntegrationTime(500);
  it = vl53l5cx.getIntegrationTime();
  Serial.print(F("   Set 500ms, readback: "));
  Serial.print(it);
  Serial.println(F(" ms"));
  report("5. Set/get 500ms", set500 && it == 500);

  // Test 6 & 7: Compare sigma at 10ms vs 100ms
  VL53L5CX_ResultsData results;

  // Measure sigma at 10ms
  vl53l5cx.setIntegrationTime(10);
  vl53l5cx.setRangingFrequency(5);
  vl53l5cx.startRanging();
  // Discard first frame
  waitAndRead(&results);
  // Average over 3 frames
  float sigma10_total = 0;
  for (uint8_t f = 0; f < 3; f++) {
    waitAndRead(&results);
    sigma10_total += avgSigma(&results, 16);
  }
  float sigma10 = sigma10_total / 3.0;
  vl53l5cx.stopRanging();

  Serial.print(F("   Avg sigma at 10ms: "));
  Serial.print(sigma10, 1);
  Serial.println(F(" mm"));
  report("6. Sigma at 10ms readable", sigma10 > 0);

  // Measure sigma at 100ms
  vl53l5cx.setIntegrationTime(100);
  vl53l5cx.setRangingFrequency(5);
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard first
  float sigma100_total = 0;
  for (uint8_t f = 0; f < 3; f++) {
    waitAndRead(&results);
    sigma100_total += avgSigma(&results, 16);
  }
  float sigma100 = sigma100_total / 3.0;
  vl53l5cx.stopRanging();

  Serial.print(F("   Avg sigma at 100ms: "));
  Serial.print(sigma100, 1);
  Serial.println(F(" mm"));
  Serial.print(F("   Sigma ratio (10ms/100ms): "));
  Serial.println(sigma10 / sigma100, 2);
  // Longer integration should have equal or lower sigma
  report("7. 100ms sigma <= 10ms sigma", sigma100 <= sigma10);

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

float avgSigma(VL53L5CX_ResultsData* results, uint8_t zones) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < zones; i++) {
    sum += results->range_sigma_mm[i];
  }
  return (float)sum / zones;
}
