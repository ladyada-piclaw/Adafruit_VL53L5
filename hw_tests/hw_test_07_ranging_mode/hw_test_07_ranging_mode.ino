/*!
 * @file hw_test_07_ranging_mode.ino
 *
 * Hardware test: VL53L5CX ranging mode (continuous vs autonomous)
 *
 * Tests set/get readback, valid ranging in both modes, and timing
 * differences: autonomous mode respects integration time independently
 * from ranging frequency, while continuous mode is frequency-driven.
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 07: Ranging Mode ==="));
  Serial.println();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &Wire, 400000)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  vl53l5cx.setResolution(16);

  // Test 1: Default mode
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
  report("2. Set/get CONTINUOUS",
         setC && mode == VL53L5CX_RANGING_MODE_CONTINUOUS);

  // Test 3: Set AUTONOMOUS
  bool setA = vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  mode = vl53l5cx.getRangingMode();
  Serial.print(F("   Set AUTONOMOUS, readback: "));
  Serial.println(mode);
  report("3. Set/get AUTONOMOUS",
         setA && mode == VL53L5CX_RANGING_MODE_AUTONOMOUS);

  // Test 4: Switch back to CONTINUOUS
  bool setBack = vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_CONTINUOUS);
  mode = vl53l5cx.getRangingMode();
  report("4. Switch back to CONTINUOUS",
         setBack && mode == VL53L5CX_RANGING_MODE_CONTINUOUS);

  VL53L5CX_ResultsData results;

  // Test 5: Range in CONTINUOUS, data valid
  vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_CONTINUOUS);
  vl53l5cx.setRangingFrequency(15);
  vl53l5cx.startRanging();
  waitAndRead(&results);
  bool readC = waitAndRead(&results);
  uint8_t validC = countValid(&results, 16);
  vl53l5cx.stopRanging();
  delay(100);
  Serial.print(F("   CONTINUOUS valid zones: "));
  Serial.print(validC);
  Serial.println(F("/16"));
  report("5. Range in CONTINUOUS", readC && validC > 0);

  // Test 6: Range in AUTONOMOUS, data valid
  vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  vl53l5cx.setIntegrationTime(50);
  vl53l5cx.setRangingFrequency(10);
  vl53l5cx.startRanging();
  waitAndRead(&results);
  bool readA = waitAndRead(&results);
  uint8_t validA = countValid(&results, 16);
  vl53l5cx.stopRanging();
  delay(100);
  Serial.print(F("   AUTONOMOUS valid zones: "));
  Serial.print(validA);
  Serial.println(F("/16"));
  report("6. Range in AUTONOMOUS", readA && validA > 0);

  // Test 7: CONTINUOUS timing — 15 Hz with 5ms integration
  // Frame interval should be ~67ms (driven by frequency, not integration)
  Serial.println();
  Serial.println(F("   --- Timing tests ---"));
  vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_CONTINUOUS);
  vl53l5cx.setRangingFrequency(15);
  vl53l5cx.setIntegrationTime(5);
  vl53l5cx.startRanging();
  float contInterval = measureFrameInterval(10);
  vl53l5cx.stopRanging();
  delay(100);
  Serial.print(F("   CONTINUOUS 15Hz/5ms integ: "));
  Serial.print(contInterval, 1);
  Serial.println(F(" ms (expect ~67ms)"));
  report("7. CONTINUOUS timing ~67ms", contInterval > 45 && contInterval < 90);

  // Test 8: AUTONOMOUS with 100ms integration at 5 Hz
  // Frame interval should be ~200ms (driven by frequency)
  vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  vl53l5cx.setIntegrationTime(100);
  vl53l5cx.setRangingFrequency(5);
  vl53l5cx.startRanging();
  float autoInterval100 = measureFrameInterval(5);
  vl53l5cx.stopRanging();
  delay(100);
  Serial.print(F("   AUTONOMOUS 5Hz/100ms integ: "));
  Serial.print(autoInterval100, 1);
  Serial.println(F(" ms (expect ~200ms)"));
  report("8. AUTONOMOUS 5Hz/100ms timing ~200ms",
         autoInterval100 > 140 && autoInterval100 < 260);

  // Test 9: AUTONOMOUS with 5ms integration at 5 Hz
  // Should still be ~200ms (frequency-driven, not integration-driven)
  vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  vl53l5cx.setIntegrationTime(5);
  vl53l5cx.setRangingFrequency(5);
  vl53l5cx.startRanging();
  float autoInterval5 = measureFrameInterval(5);
  vl53l5cx.stopRanging();
  delay(100);
  Serial.print(F("   AUTONOMOUS 5Hz/5ms integ: "));
  Serial.print(autoInterval5, 1);
  Serial.println(F(" ms (expect ~200ms)"));
  report("9. AUTONOMOUS 5Hz/5ms timing ~200ms",
         autoInterval5 > 140 && autoInterval5 < 260);

  // Test 10: AUTONOMOUS with long integration exceeding period
  // 10 Hz = 100ms period, but 200ms integration should stretch it
  vl53l5cx.setRangingMode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  vl53l5cx.setIntegrationTime(200);
  vl53l5cx.setRangingFrequency(10);
  vl53l5cx.startRanging();
  float autoLong = measureFrameInterval(3);
  vl53l5cx.stopRanging();
  delay(100);
  Serial.print(F("   AUTONOMOUS 10Hz/200ms integ: "));
  Serial.print(autoLong, 1);
  Serial.println(F(" ms (expect >= 200ms)"));
  report("10. AUTONOMOUS integ exceeds period", autoLong > 160);

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

uint8_t countValid(VL53L5CX_ResultsData* results, uint8_t zones) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < zones; i++) {
    if (results->distance_mm[i] > 0 && results->distance_mm[i] < 4000)
      count++;
  }
  return count;
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

float measureFrameInterval(uint8_t numFrames) {
  VL53L5CX_ResultsData results;
  while (vl53l5cx.isDataReady()) {
    vl53l5cx.getRangingData(&results);
  }
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
