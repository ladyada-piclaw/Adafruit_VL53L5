/*!
 * @file hw_test_11_detection_thresholds.ino
 *
 * Hardware test: VL53L5CX Detection Thresholds
 *
 * Tests the detection threshold API:
 *  1. Configure threshold for zone 0: distance < 500mm
 *  2. Set thresholds via setDetectionThresholds()
 *  3. Enable thresholds via setDetectionThresholdsEnable()
 *  4. Read back enabled state and verify
 *  5. Read back threshold values and verify
 *  6. Verify INT pin fires when threshold triggers
 *
 * Wiring: VL53L5CX INT -> A2 (QT Py SAMD21)
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_VL53L5CX.h>
#include <Wire.h>

// Board-specific I2C setup (from hw_test_helper.h)
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) ||                               \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) ||                                  \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) ||                          \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3)
#define HW_TEST_WIRE Wire1
#define HW_TEST_I2C_INIT()                                                     \
  do {                                                                         \
    Wire1.begin(SDA1, SCL1);                                                   \
    Wire1.setClock(400000);                                                    \
  } while (0)
#else
#define HW_TEST_WIRE Wire
#define HW_TEST_I2C_INIT()                                                     \
  do {                                                                         \
    Wire.begin();                                                              \
    Wire.setClock(400000);                                                     \
  } while (0)
#endif

#define INT_PIN A2

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;
VL53L5CX_DetectionThresholds thresholds[VL53L5CX_NB_THRESHOLDS];

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("=== HW Test 11: Detection Thresholds ==="));
  Serial.println();

  pinMode(INT_PIN, INPUT_PULLUP);
  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE)) {
    Serial.println(F("Init failed!"));
    while (1)
      delay(10);
  }

  // Set 8x8 resolution, 15Hz
  vl53l5cx.setResolution(64);
  vl53l5cx.setRangingFrequency(15);

  // Must stop ranging before configuring thresholds
  vl53l5cx.stopRanging();

  // Initialize all thresholds to zero
  memset(thresholds, 0, sizeof(thresholds));

  // Configure zone 0: trigger when distance <= 500mm
  thresholds[0].zone_num = 0;
  thresholds[0].measurement = VL53L5CX_DISTANCE_MM;
  thresholds[0].type = VL53L5CX_LESS_THAN_EQUAL_MIN_CHECKER;
  thresholds[0].param_low_thresh = 500;
  thresholds[0].param_high_thresh = 500;
  thresholds[0].mathematic_operation = VL53L5CX_OPERATION_OR;

  // Terminate the threshold list
  thresholds[1].zone_num = VL53L5CX_LAST_THRESHOLD;

  // Test 1: Set detection thresholds
  bool setOk = vl53l5cx.setDetectionThresholds(thresholds);
  report("1. setDetectionThresholds()", setOk);

  // Test 2: Enable detection thresholds
  bool enableOk = vl53l5cx.setDetectionThresholdsEnable(true);
  report("2. setDetectionThresholdsEnable(true)", enableOk);

  // Test 3: Read back enabled state
  bool isEnabled = vl53l5cx.getDetectionThresholdsEnable();
  report("3. getDetectionThresholdsEnable() == true", isEnabled);

  // Test 4: Read back thresholds and verify zone 0 values
  VL53L5CX_DetectionThresholds readBack[VL53L5CX_NB_THRESHOLDS];
  bool getOk = vl53l5cx.getDetectionThresholds(readBack);
  Serial.print(F("   Read back: zone_num="));
  Serial.print(readBack[0].zone_num);
  Serial.print(F(", measurement="));
  Serial.print(readBack[0].measurement);
  Serial.print(F(", type="));
  Serial.print(readBack[0].type);
  Serial.print(F(", low="));
  Serial.print(readBack[0].param_low_thresh);
  Serial.print(F(", high="));
  Serial.println(readBack[0].param_high_thresh);

  bool valuesMatch = getOk && (readBack[0].zone_num == 0) &&
                     (readBack[0].measurement == VL53L5CX_DISTANCE_MM) &&
                     (readBack[0].type == VL53L5CX_LESS_THAN_EQUAL_MIN_CHECKER) &&
                     (readBack[0].param_low_thresh == 500);
  report("4. getDetectionThresholds() values match", valuesMatch);

  // Start ranging
  vl53l5cx.startRanging();

  Serial.println();
  Serial.println(F("   Running 50 iterations with threshold monitoring..."));
  Serial.println(F("   Zone 0 threshold: distance <= 500mm triggers INT"));
  Serial.println();
}

void loop() {
  static uint8_t iteration = 0;
  static uint8_t intFiredCount = 0;
  static uint8_t thresholdCorrelationOk = 0;

  if (iteration >= 50) {
    // Test 5: Check INT correlation with threshold
    Serial.println();
    Serial.print(F("   INT fired "));
    Serial.print(intFiredCount);
    Serial.println(F(" times"));
    Serial.print(F("   Threshold correlation correct: "));
    Serial.print(thresholdCorrelationOk);
    Serial.println(F("/50"));

    // Pass if INT behavior correlated with distance at least 80% of the time
    report("5. INT fires correlate with threshold (>=40/50)",
           thresholdCorrelationOk >= 40);

    // Summary
    printSummary();
    while (1)
      delay(1000);
  }

  // Check if data is ready
  if (vl53l5cx.isDataReady()) {
    // Check INT pin BEFORE reading (should be LOW if threshold triggered)
    bool intFired = (digitalRead(INT_PIN) == LOW);
    if (intFired) {
      intFiredCount++;
    }

    // Read the data
    vl53l5cx.getRangingData(&results);

    // Check zone 0 distance
    int16_t zone0dist = results.distance_mm[0];
    bool shouldTrigger = (zone0dist <= 500 && zone0dist > 0);

    // Check correlation: INT should fire when distance <= 500mm
    if ((intFired && shouldTrigger) || (!intFired && !shouldTrigger)) {
      thresholdCorrelationOk++;
    }

    Serial.print(F("   ["));
    Serial.print(iteration);
    Serial.print(F("] Zone 0: "));
    Serial.print(zone0dist);
    Serial.print(F("mm, INT: "));
    Serial.print(intFired ? F("FIRED") : F("--"));
    Serial.print(F(", Expected: "));
    Serial.println(shouldTrigger ? F("TRIGGER") : F("no trigger"));

    iteration++;
  }

  delay(10);
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

void printSummary() {
  Serial.println();
  Serial.println(F("=== SUMMARY ==="));
  Serial.print(passed);
  Serial.print(F(" passed, "));
  Serial.print(failed);
  Serial.println(F(" failed"));
  Serial.println();
  if (failed == 0) {
    Serial.println(F("PASS: hw_test_11"));
  } else {
    Serial.println(F("FAIL: hw_test_11"));
  }
}
