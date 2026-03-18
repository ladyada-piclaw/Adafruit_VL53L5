/*!
 * @file hw_test_05_sharpener.ino
 *
 * Hardware test: VL53L5CX sharpener percent set/get
 *
 * Tests:
 *  1. Set sharpener 0% (off), readback matches
 *  2. Set sharpener 5%, readback matches
 *  3. Set sharpener 14%, readback matches
 *  4. Set sharpener 50%, readback matches
 *  5. Set sharpener 99%, readback matches
 *  6. Range with sharpener 0%, measure distance spread
 *  7. Range with sharpener 50%, measure distance spread
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>
#include "hw_test_helper.h"

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;



// Compute standard deviation of distance_mm across zones

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("=== HW Test 05: Sharpener ==="));
  Serial.println();

  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE, 1000000)) {
    Serial.println(F("Init failed!"));
    while (1) delay(10);
  }

  // Use 8x8 for more zones to see sharpener effect
  vl53l5cx.setResolution(64);
  vl53l5cx.setRangingFrequency(15);

  // Test 1: 0% (off)
  bool set0 = vl53l5cx.setSharpenerPercent(0);
  uint8_t sharp = vl53l5cx.getSharpenerPercent();
  Serial.print(F("   Set 0%, readback: "));
  Serial.print(sharp);
  Serial.println(F("%"));
  report("1. Set/get 0% (off)", set0 && sharp == 0);

  // Test 2: 5% (ST driver readback may be +/-1 due to internal rounding)
  bool set5 = vl53l5cx.setSharpenerPercent(5);
  sharp = vl53l5cx.getSharpenerPercent();
  Serial.print(F("   Set 5%, readback: "));
  Serial.print(sharp);
  Serial.println(F("%"));
  report("2. Set/get 5% (+/-1)", set5 && sharp >= 4 && sharp <= 6);

  // Test 3: 14%
  bool set14 = vl53l5cx.setSharpenerPercent(14);
  sharp = vl53l5cx.getSharpenerPercent();
  Serial.print(F("   Set 14%, readback: "));
  Serial.print(sharp);
  Serial.println(F("%"));
  report("3. Set/get 14% (+/-1)", set14 && sharp >= 13 && sharp <= 15);

  // Test 4: 50%
  bool set50 = vl53l5cx.setSharpenerPercent(50);
  sharp = vl53l5cx.getSharpenerPercent();
  Serial.print(F("   Set 50%, readback: "));
  Serial.print(sharp);
  Serial.println(F("%"));
  report("4. Set/get 50% (+/-1)", set50 && sharp >= 49 && sharp <= 51);

  // Test 5: 99%
  bool set99 = vl53l5cx.setSharpenerPercent(99);
  sharp = vl53l5cx.getSharpenerPercent();
  Serial.print(F("   Set 99%, readback: "));
  Serial.print(sharp);
  Serial.println(F("%"));
  report("5. Set/get 99% (+/-1)", set99 && sharp >= 98 && sharp <= 99);

  VL53L5CX_ResultsData results;

  // Test 6: Measure spread with sharpener off
  vl53l5cx.setSharpenerPercent(0);
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard first
  float stddev0_total = 0;
  for (uint8_t f = 0; f < 3; f++) {
    waitAndRead(&results);
    stddev0_total += distanceStdDev(&results, 64);
  }
  float stddev0 = stddev0_total / 3.0;
  vl53l5cx.stopRanging();
  Serial.print(F("   StdDev at 0%: "));
  Serial.print(stddev0, 1);
  Serial.println(F(" mm"));
  report("6. Range with sharpener 0%", stddev0 >= 0);

  // Test 7: Measure spread with sharpener 50%
  vl53l5cx.setSharpenerPercent(50);
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard first
  float stddev50_total = 0;
  for (uint8_t f = 0; f < 3; f++) {
    waitAndRead(&results);
    stddev50_total += distanceStdDev(&results, 64);
  }
  float stddev50 = stddev50_total / 3.0;
  vl53l5cx.stopRanging();
  Serial.print(F("   StdDev at 50%: "));
  Serial.print(stddev50, 1);
  Serial.println(F(" mm"));
  Serial.print(F("   Ratio (0%/50%): "));
  if (stddev50 > 0)
    Serial.println(stddev0 / stddev50, 2);
  else
    Serial.println(F("N/A"));
  report("7. Range with sharpener 50%", stddev50 >= 0);

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

float distanceStdDev(VL53L5CX_ResultsData *results, uint8_t zones) {
  // First pass: mean
  float sum = 0;
  uint8_t valid = 0;
  for (uint8_t i = 0; i < zones; i++) {
    if (results->distance_mm[i] > 0 && results->distance_mm[i] < 4000) {
      sum += results->distance_mm[i];
      valid++;
    }
  }
  if (valid == 0) return -1;
  float mean = sum / valid;

  // Second pass: variance
  float varSum = 0;
  for (uint8_t i = 0; i < zones; i++) {
    if (results->distance_mm[i] > 0 && results->distance_mm[i] < 4000) {
      float diff = results->distance_mm[i] - mean;
      varSum += diff * diff;
    }
  }
  return sqrt(varSum / valid);
}
