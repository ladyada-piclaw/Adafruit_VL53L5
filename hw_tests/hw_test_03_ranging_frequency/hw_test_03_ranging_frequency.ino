/*!
 * @file hw_test_03_ranging_frequency.ino
 *
 * Hardware test: VL53L5CX ranging frequency set/get and timing
 *
 * Tests:
 *  1. Set frequency 1 Hz, readback matches
 *  2. Set frequency 5 Hz, readback matches
 *  3. Set frequency 15 Hz, readback matches
 *  4. Set frequency 30 Hz, readback matches (4x4 only)
 *  5. Set frequency 60 Hz, readback matches (4x4 only)
 *  6. Measure actual frame interval at 15 Hz (~67ms expected)
 *  7. Measure actual frame interval at 5 Hz (~200ms expected)
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>

#include "hw_test_helper.h"

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

// Measure average interval between frames over N frames

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 03: Ranging Frequency ==="));
  Serial.println();

  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE, 1000000)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  // Use 4x4 for max frequency range (up to 60 Hz)
  vl53l5cx.setResolution(16);

  // Test 1: 1 Hz
  bool set1 = vl53l5cx.setRangingFrequency(1);
  uint8_t freq = vl53l5cx.getRangingFrequency();
  Serial.print(F("   Set 1 Hz, readback: "));
  Serial.print(freq);
  Serial.println(F(" Hz"));
  report("1. Set/get 1 Hz", set1 && freq == 1);

  // Test 2: 5 Hz
  bool set5 = vl53l5cx.setRangingFrequency(5);
  freq = vl53l5cx.getRangingFrequency();
  Serial.print(F("   Set 5 Hz, readback: "));
  Serial.print(freq);
  Serial.println(F(" Hz"));
  report("2. Set/get 5 Hz", set5 && freq == 5);

  // Test 3: 15 Hz
  bool set15 = vl53l5cx.setRangingFrequency(15);
  freq = vl53l5cx.getRangingFrequency();
  Serial.print(F("   Set 15 Hz, readback: "));
  Serial.print(freq);
  Serial.println(F(" Hz"));
  report("3. Set/get 15 Hz", set15 && freq == 15);

  // Test 4: 30 Hz
  bool set30 = vl53l5cx.setRangingFrequency(30);
  freq = vl53l5cx.getRangingFrequency();
  Serial.print(F("   Set 30 Hz, readback: "));
  Serial.print(freq);
  Serial.println(F(" Hz"));
  report("4. Set/get 30 Hz", set30 && freq == 30);

  // Test 5: 60 Hz
  bool set60 = vl53l5cx.setRangingFrequency(60);
  freq = vl53l5cx.getRangingFrequency();
  Serial.print(F("   Set 60 Hz, readback: "));
  Serial.print(freq);
  Serial.println(F(" Hz"));
  report("5. Set/get 60 Hz", set60 && freq == 60);

  // Test 6: Measure actual timing at 15 Hz
  vl53l5cx.setRangingFrequency(15);
  vl53l5cx.startRanging();
  float interval15 = measureFrameInterval(10);
  vl53l5cx.stopRanging();
  Serial.print(F("   15 Hz avg interval: "));
  Serial.print(interval15, 1);
  Serial.println(F(" ms (expect ~67ms)"));
  // Allow 30% tolerance
  report("6. 15 Hz timing (~67ms)", interval15 > 45 && interval15 < 90);

  // Test 7: Measure actual timing at 5 Hz
  vl53l5cx.setRangingFrequency(5);
  vl53l5cx.startRanging();
  float interval5 = measureFrameInterval(5);
  vl53l5cx.stopRanging();
  Serial.print(F("   5 Hz avg interval: "));
  Serial.print(interval5, 1);
  Serial.println(F(" ms (expect ~200ms)"));
  // Allow 30% tolerance
  report("7. 5 Hz timing (~200ms)", interval5 > 140 && interval5 < 260);

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
