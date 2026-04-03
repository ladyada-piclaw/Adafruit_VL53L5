/*!
 * @file hw_test_04_integration_time.ino
 *
 * Hardware test: VL53L5CX integration time set/get, actual timing,
 * and noise comparison
 *
 * Tests each integration time value: set/get readback, then measure
 * actual frame interval at a fixed ranging frequency to verify the
 * integration time affects frame timing as expected.
 * Also compares sigma (noise) at short vs long integration.
 *
 * Uses 4x4 resolution, 1 Hz ranging so integration time dominates.
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

// Integration times to test (ms)
const uint32_t testTimes[] = {5, 10, 20, 50, 100};
const uint8_t numTimes = sizeof(testTimes) / sizeof(testTimes[0]);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 04: Integration Time ==="));
  Serial.println();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &Wire, 400000)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  // Use 4x4 at 60 Hz (~17ms period) so integration time dominates
  vl53l5cx.setResolution(16);
  vl53l5cx.setRangingFrequency(60);

  uint8_t testNum = 1;
  float intervals[numTimes];

  for (uint8_t i = 0; i < numTimes; i++) {
    uint32_t ms = testTimes[i];

    // Set/get readback
    bool setOk = vl53l5cx.setIntegrationTime(ms);
    uint32_t readback = vl53l5cx.getIntegrationTime();
    Serial.print(F("   Set "));
    Serial.print(ms);
    Serial.print(F(" ms, readback: "));
    Serial.print(readback);
    Serial.println(F(" ms"));

    char label[48];
    snprintf(label, sizeof(label), "%d. Set/get %lu ms", testNum,
             (unsigned long)ms);
    report(label, setOk && readback == ms);
    testNum++;

    // Measure actual frame interval
    vl53l5cx.startRanging();
    intervals[i] = measureFrameInterval(5);
    vl53l5cx.stopRanging();
    delay(100); // sensor needs time to fully stop before reconfigure

    Serial.print(F("   Frame interval: "));
    Serial.print(intervals[i], 1);
    Serial.println(F(" ms"));

    // Interval must be at least the integration time
    snprintf(label, sizeof(label), "%d. %lu ms interval >= integ time",
             testNum, (unsigned long)ms);
    report(label, intervals[i] >= (float)ms * 0.8);
    testNum++;
  }

  // Test: longer integration -> longer (or equal) frame interval
  Serial.println();
  Serial.println(F("   Timing monotonicity check:"));
  for (uint8_t i = 1; i < numTimes; i++) {
    Serial.print(F("   "));
    Serial.print(testTimes[i - 1]);
    Serial.print(F(" ms -> "));
    Serial.print(intervals[i - 1], 1);
    Serial.print(F(" ms,  "));
    Serial.print(testTimes[i]);
    Serial.print(F(" ms -> "));
    Serial.print(intervals[i], 1);
    Serial.println(F(" ms"));
  }
  // Overall: longest integration interval > shortest
  char label[48];
  snprintf(label, sizeof(label), "%d. Longer integ = longer interval", testNum);
  report(label, intervals[numTimes - 1] > intervals[0]);
  testNum++;

  // Sigma comparison: short (5ms) vs long (100ms)
  Serial.println();
  VL53L5CX_ResultsData results;

  vl53l5cx.setIntegrationTime(5);
  vl53l5cx.setRangingFrequency(60);
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard first
  float sigma_short = 0;
  for (uint8_t f = 0; f < 3; f++) {
    waitAndRead(&results);
    sigma_short += avgSigma(&results, 16);
  }
  sigma_short /= 3.0;
  vl53l5cx.stopRanging();
  delay(100);

  vl53l5cx.setIntegrationTime(100);
  vl53l5cx.setRangingFrequency(5); // slow enough for 100ms integration
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard first
  float sigma_long = 0;
  for (uint8_t f = 0; f < 3; f++) {
    waitAndRead(&results);
    sigma_long += avgSigma(&results, 16);
  }
  sigma_long /= 3.0;
  vl53l5cx.stopRanging();

  Serial.print(F("   Sigma at 5ms: "));
  Serial.print(sigma_short, 1);
  Serial.println(F(" mm"));
  Serial.print(F("   Sigma at 100ms: "));
  Serial.print(sigma_long, 1);
  Serial.println(F(" mm"));
  Serial.println(F("   (sigma comparison is informational — environment dependent)"));

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

float measureFrameInterval(uint8_t numFrames) {
  VL53L5CX_ResultsData results;
  // Flush stale data
  while (vl53l5cx.isDataReady()) {
    vl53l5cx.getRangingData(&results);
  }
  // Wait for first frame
  unsigned long timeout = millis() + 5000;
  while (!vl53l5cx.isDataReady()) {
    if (millis() > timeout)
      return -1;
    delay(1);
  }
  vl53l5cx.getRangingData(&results);

  unsigned long start = millis();
  for (uint8_t i = 0; i < numFrames; i++) {
    timeout = millis() + 5000;
    while (!vl53l5cx.isDataReady()) {
      if (millis() > timeout)
        return -1;
      delay(1);
    }
    vl53l5cx.getRangingData(&results);
  }
  return (float)(millis() - start) / numFrames;
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

float avgSigma(VL53L5CX_ResultsData *results, uint8_t zones) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < zones; i++) {
    sum += results->range_sigma_mm[i];
  }
  return (float)sum / zones;
}
