/*!
 * @file hw_test_03_ranging_frequency.ino
 *
 * Hardware test: VL53L5CX ranging frequency set/get and actual timing
 *
 * Tests each supported frequency: set/get readback, then measure
 * actual frame interval and verify it's within 30% of expected.
 *
 * Uses 4x4 resolution for max frequency range (up to 60 Hz).
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

// Frequencies to test, expected intervals, and frame counts
const uint8_t testFreqs[] = {1, 5, 15, 30, 60};
const float expectedMs[] = {1000.0, 200.0, 66.7, 33.3, 16.7};
const uint8_t frameCounts[] = {3, 5, 10, 15, 20};
const uint8_t numTests = sizeof(testFreqs) / sizeof(testFreqs[0]);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 03: Ranging Frequency ==="));
  Serial.println();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &Wire, 400000)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  // Use 4x4 for max frequency range (up to 60 Hz)
  vl53l5cx.setResolution(16);

  uint8_t testNum = 1;

  for (uint8_t i = 0; i < numTests; i++) {
    uint8_t hz = testFreqs[i];
    float expectMs = expectedMs[i];
    uint8_t frames = frameCounts[i];

    // Set/get readback
    bool setOk = vl53l5cx.setRangingFrequency(hz);
    uint8_t readback = vl53l5cx.getRangingFrequency();
    Serial.print(F("   Set "));
    Serial.print(hz);
    Serial.print(F(" Hz, readback: "));
    Serial.print(readback);
    Serial.println(F(" Hz"));

    char label[48];
    snprintf(label, sizeof(label), "%d. Set/get %d Hz", testNum, hz);
    report(label, setOk && readback == hz);
    testNum++;

    // Measure actual interval
    vl53l5cx.startRanging();
    float interval = measureFrameInterval(frames);
    vl53l5cx.stopRanging();
    delay(100); // sensor needs time to fully stop before reconfigure

    Serial.print(F("   Measured interval: "));
    Serial.print(interval, 1);
    Serial.print(F(" ms (expect ~"));
    Serial.print(expectMs, 1);
    Serial.println(F(" ms)"));

    float lo = expectMs * 0.7;
    float hi = expectMs * 1.3;
    snprintf(label, sizeof(label), "%d. %d Hz timing", testNum, hz);
    report(label, interval > lo && interval < hi);
    testNum++;
  }

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
  // Flush stale data
  VL53L5CX_ResultsData results;
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
  unsigned long elapsed = millis() - start;
  return (float)elapsed / numFrames;
}
