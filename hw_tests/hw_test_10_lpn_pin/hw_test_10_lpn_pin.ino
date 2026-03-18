/*!
 * @file hw_test_10_lpn_pin.ino
 *
 * Hardware test: VL53L5CX LPn (reset/enable) pin
 *
 * LPn is active-low: LOW = sensor held in reset, HIGH = sensor enabled.
 * Same function as LPn/XSHUT on other ST ToF sensors.
 *
 * Tests:
 *  1. Sensor responds on I2C with LPn HIGH (default)
 *  2. begin() succeeds with LPn HIGH
 *  3. Sensor disappears from I2C when LPn pulled LOW
 *  4. Sensor reappears on I2C when LPn released HIGH
 *  5. begin() succeeds after reset cycle
 *  6. Ranging works after reset cycle
 *
 * Wiring: VL53L5CX LPn -> A1
 */

#include <Adafruit_VL53L5CX.h>
#include "hw_test_helper.h"

#ifdef ESP32
#define LPN_PIN 32
#else
#define LPN_PIN A1
#endif

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("=== HW Test 10: LPn Pin ==="));
  Serial.println();

  // Let LPn float (breakout has pullup) to start
  pinMode(LPN_PIN, INPUT);
  delay(100);

  HW_TEST_I2C_INIT();

  // Test 1: Sensor on I2C with LPn HIGH
  bool found = i2cPresent(0x29);
  Serial.print(F("   I2C scan (LPn HIGH): "));
  Serial.println(found ? F("found") : F("not found"));
  report("1. Sensor on I2C (LPn HIGH)", found);

  // Test 2: begin() succeeds
  bool initOk = vl53l5cx.begin(0x29, &HW_TEST_WIRE, 1000000);
  report("2. begin() succeeds", initOk);
  if (!initOk) {
    Serial.println(F("Init failed, cannot continue."));
    while (1) delay(10);
  }

  // Test 3: Pull LPn LOW — sensor should disappear
  pinMode(LPN_PIN, OUTPUT);
  digitalWrite(LPN_PIN, LOW);
  delay(100);
  bool gone = !i2cPresent(0x29);
  Serial.print(F("   I2C scan (LPn LOW): "));
  Serial.println(gone ? F("not found (correct)") : F("still found!"));
  report("3. Sensor gone (LPn LOW)", gone);

  // Test 4: Release LPn (back to input/pullup) — sensor should reappear
  pinMode(LPN_PIN, INPUT);
  delay(100);
  bool back = i2cPresent(0x29);
  Serial.print(F("   I2C scan (LPn HIGH again): "));
  Serial.println(back ? F("found") : F("not found"));
  report("4. Sensor back (LPn HIGH)", back);

  // Test 5: begin() succeeds after reset
  bool reinitOk = vl53l5cx.begin(0x29, &HW_TEST_WIRE, 1000000);
  report("5. begin() after reset", reinitOk);

  // Test 6: Ranging works after reset
  if (reinitOk) {
    vl53l5cx.setRangingFrequency(15);
    vl53l5cx.startRanging();
    unsigned long t = millis();
    bool gotData = false;
    while (millis() - t < 3000) {
      if (vl53l5cx.isDataReady()) {
        VL53L5CX_ResultsData results;
        gotData = vl53l5cx.getRangingData(&results);
        if (gotData) {
          Serial.print(F("   Distance[0]: "));
          Serial.print(results.distance_mm[0]);
          Serial.println(F(" mm"));
        }
        break;
      }
      delay(5);
    }
    vl53l5cx.stopRanging();
    report("6. Ranging after reset", gotData);
  } else {
    report("6. Ranging after reset", false);
  }

  // Release LPn (leave sensor enabled)
  pinMode(LPN_PIN, INPUT);

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

bool i2cPresent(uint8_t addr) {
  HW_TEST_WIRE.beginTransmission(addr);
  return (HW_TEST_WIRE.endTransmission() == 0);
}
