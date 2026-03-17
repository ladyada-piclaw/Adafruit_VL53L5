/*!
 * @file hw_test_07_ranging_mode.ino
 *
 * Hardware test: VL53L5CX ranging mode (continuous vs autonomous)
 *
 * Tests:
 *  1. Default ranging mode readback
 *  2. Set CONTINUOUS (1), readback matches
 *  3. Set AUTONOMOUS (3), readback matches
 *  4. Switch back to CONTINUOUS, readback matches
 *  5. Range in CONTINUOUS mode, data valid
 *  6. Range in AUTONOMOUS mode, data valid
 *  7. Autonomous respects integration time setting
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>
#include "../hw_test_helper.h"

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

float measureFrameInterval(uint8_t numFrames) {
  VL53L5CX_ResultsData results;
  // Flush
  while (vl53l5cx.isDataReady()) {
    vl53l5cx.getRangingData(&results);
  }
  // Wait for first
  unsigned long timeout = millis() + 5000;
  while (!vl53l5cx.isDataReady()) {
    if (millis() > timeout) return -1;
    delay(1);
  }
  vl53l5cx.getRangingData(&results);

  unsigned long start = millis();
  for (uint8_t i = 0; i < numFrames; i++) {
    timeout = millis() + 5000;
    while (!vl53l5cx.isDataReady()) {
      if (millis() > timeout) return -1;
      delay(1);
    }
    vl53l5cx.getRangingData(&results);
  }
  return (float)(millis() - start) / numFrames;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("=== HW Test 07: Ranging Mode ==="));
  Serial.println();

  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE)) {
    Serial.println(F("Init failed!"));
    while (1) delay(10);
  }

  vl53l5cx.setResolution(16);
  vl53l5cx.setRangingFrequency(10);

  // Test 1: Default
  uint8_t mode = vl53l5cx.getRangingMode();
  Serial.print(F("   Default mode: "));
  Serial.println(mode);
  report("1. Default ranging mode readable",
         mode == VL53L5CX_RANGING_MODE_CONTINUOUS ||
         mode == VL53L5CX_RANGING_MODE_AUTONOMOUS);

  // Test 2: Set CONTINUOUS
  bool setC = vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_CONTINUOUS);
  mode = vl53l5cx.getRangingMode();
  Serial.print(F("   Set CONTINUOUS, readback: "));
  Serial.println(mode);
  report("2. Set/get CONTINUOUS (1)", setC && mode == VL53L5CX_RANGING_MODE_CONTINUOUS);

  // Test 3: Set AUTONOMOUS
  bool setA = vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  mode = vl53l5cx.getRangingMode();
  Serial.print(F("   Set AUTONOMOUS, readback: "));
  Serial.println(mode);
  report("3. Set/get AUTONOMOUS (3)", setA && mode == VL53L5CX_RANGING_MODE_AUTONOMOUS);

  // Test 4: Switch back
  bool setBack = vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_CONTINUOUS);
  mode = vl53l5cx.getRangingMode();
  Serial.print(F("   Back to CONTINUOUS, readback: "));
  Serial.println(mode);
  report("4. Switch back to CONTINUOUS", setBack && mode == VL53L5CX_RANGING_MODE_CONTINUOUS);

  VL53L5CX_ResultsData results;

  // Test 5: Range in CONTINUOUS
  vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_CONTINUOUS);
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard
  bool readC = waitAndRead(&results);
  uint8_t validC = 0;
  if (readC) {
    for (uint8_t i = 0; i < 16; i++) {
      if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000)
        validC++;
    }
    Serial.print(F("   CONTINUOUS valid zones: "));
    Serial.print(validC);
    Serial.println(F("/16"));
  }
  vl53l5cx.stopRanging();
  report("5. Range in CONTINUOUS", readC && validC > 0);

  // Test 6: Range in AUTONOMOUS
  vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  vl53l5cx.setIntegrationTime(50);
  vl53l5cx.startRanging();
  waitAndRead(&results); // discard
  bool readA = waitAndRead(&results);
  uint8_t validA = 0;
  if (readA) {
    for (uint8_t i = 0; i < 16; i++) {
      if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000)
        validA++;
    }
    Serial.print(F("   AUTONOMOUS valid zones: "));
    Serial.print(validA);
    Serial.println(F("/16"));
  }
  vl53l5cx.stopRanging();
  report("6. Range in AUTONOMOUS", readA && validA > 0);

  // Test 7: Autonomous timing reflects integration time
  // In autonomous mode, frame interval >= integration time
  vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  vl53l5cx.setIntegrationTime(100);
  vl53l5cx.setRangingFrequency(10);
  vl53l5cx.startRanging();
  float interval = measureFrameInterval(5);
  vl53l5cx.stopRanging();
  Serial.print(F("   Autonomous 100ms int, interval: "));
  Serial.print(interval, 1);
  Serial.println(F(" ms"));
  // Frame interval should be >= 100ms
  report("7. Autonomous interval >= 100ms", interval >= 90);

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
