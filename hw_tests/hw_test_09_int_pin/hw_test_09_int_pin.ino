/*!
 * @file hw_test_09_int_pin.ino
 *
 * Hardware test: VL53L5CX INT (interrupt) pin
 *
 * INT is active-low, directly connected to sensor GPIO output.
 * Asserted (LOW) when data is ready, deasserted (HIGH) after data read.
 *
 * Tests:
 *  1. INT pin is HIGH before ranging starts
 *  2. INT pin goes LOW when data is ready
 *  3. INT pin returns HIGH after data is read
 *  4. INT fires on every frame (check 5 frames)
 *  5. INT stays HIGH when ranging is stopped
 *
 * Wiring: VL53L5CX INT -> A0
 */

#include <Adafruit_VL53L5CX.h>
#include "hw_test_helper.h"

#ifdef ESP32
#define INT_PIN 14
#else
#define INT_PIN 2
#endif

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

// Wait for INT pin to reach target state, with timeout
bool waitForPin(uint8_t state, unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (digitalRead(INT_PIN) == state) {
      return true;
    }
    delay(1);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("=== HW Test 09: INT Pin ==="));
  Serial.println();

  pinMode(INT_PIN, INPUT_PULLUP);
  HW_TEST_I2C_INIT();

  Serial.println(F("   Initializing sensor..."));
  if (!vl53l5cx.begin(0x29, &HW_TEST_WIRE)) {
    Serial.println(F("Init failed!"));
    while (1) delay(10);
  }

  // Test 1: INT high before ranging
  bool intHigh = (digitalRead(INT_PIN) == HIGH);
  Serial.print(F("   INT pin before ranging: "));
  Serial.println(intHigh ? F("HIGH") : F("LOW"));
  report("1. INT HIGH before ranging", intHigh);

  // Start ranging at 15 Hz
  vl53l5cx.setRangingFrequency(15);
  vl53l5cx.startRanging();

  // Test 2: INT goes LOW when data ready
  bool wentLow = waitForPin(LOW, 3000);
  Serial.print(F("   INT after data ready: "));
  Serial.println(digitalRead(INT_PIN) == LOW ? F("LOW") : F("HIGH"));
  report("2. INT goes LOW on data ready", wentLow);

  // Test 3: INT returns HIGH after data read
  VL53L5CX_ResultsData results;
  vl53l5cx.getRangingData(&results);
  // Small delay for INT to deassert
  delay(5);
  bool backHigh = (digitalRead(INT_PIN) == HIGH);
  Serial.print(F("   INT after data read: "));
  Serial.println(backHigh ? F("HIGH") : F("LOW"));
  report("3. INT HIGH after data read", backHigh);

  // Test 4: INT fires on every frame (5 frames)
  uint8_t intFired = 0;
  for (int frame = 0; frame < 5; frame++) {
    // Wait for INT to go LOW (data ready)
    if (waitForPin(LOW, 1000)) {
      intFired++;
      // Read data to clear the interrupt
      vl53l5cx.getRangingData(&results);
      // Wait for INT to deassert (HIGH) before next frame
      waitForPin(HIGH, 100);
    }
  }
  Serial.print(F("   INT fired for "));
  Serial.print(intFired);
  Serial.println(F("/5 frames"));
  report("4. INT fires on frames (>=4/5)", intFired >= 4);

  // Stop ranging
  vl53l5cx.stopRanging();

  // Test 5: INT stays HIGH after stop
  delay(500); // Let any pending interrupt clear
  bool staysHigh = (digitalRead(INT_PIN) == HIGH);
  Serial.print(F("   INT after stop: "));
  Serial.println(staysHigh ? F("HIGH") : F("LOW"));
  report("5. INT HIGH after stop", staysHigh);

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
