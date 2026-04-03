/*!
 * @file hw_test_01_init_alive.ino
 *
 * Hardware test: VL53L5CX initialization and basic readback
 *
 * Tests:
 *  1. I2C scan finds sensor at 0x29
 *  2. begin() succeeds (firmware upload)
 *  3. Default resolution readback
 *  4. Default ranging frequency readback
 *  5. Default integration time readback
 *  6. Default sharpener readback
 *  7. Start ranging succeeds
 *  8. Data ready within timeout
 *  9. getRangingData succeeds
 * 10. Distance values are sane (not all zero, not all 0xFFFF)
 *
 * Connect VL53L5CX via STEMMA QT / I2C. No extra pins needed.
 */

#include <Adafruit_VL53L5CX.h>
#include <Wire.h>

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 01: Init & Alive ==="));
  Serial.println();

  // Test 1: I2C scan
  Wire.begin();
  Wire.beginTransmission(0x29);
  bool i2cFound = (Wire.endTransmission() == 0);
  report("1. I2C device at 0x29", i2cFound);
  if (!i2cFound) {
    Serial.println(F("Sensor not found, cannot continue."));
    while (1)
      delay(10);
  }

  // Test 2: begin()
  Serial.println(F("   Initializing sensor (up to 10s)..."));
  bool initOk = vl53l5cx.begin(0x29, &Wire, 400000);
  report("2. begin() succeeds", initOk);
  if (!initOk) {
    Serial.println(F("Init failed, cannot continue."));
    while (1)
      delay(10);
  }

  // Test 3: Default resolution
  uint8_t res = vl53l5cx.getResolution();
  Serial.print(F("   Resolution: "));
  Serial.println(res);
  report("3. Default resolution readback", res == 16 || res == 64);

  // Test 4: Default ranging frequency
  uint8_t freq = vl53l5cx.getRangingFrequency();
  Serial.print(F("   Ranging frequency: "));
  Serial.print(freq);
  Serial.println(F(" Hz"));
  report("4. Default ranging freq readback", freq > 0 && freq <= 60);

  // Test 5: Default integration time
  uint32_t intTime = vl53l5cx.getIntegrationTime();
  Serial.print(F("   Integration time: "));
  Serial.print(intTime);
  Serial.println(F(" ms"));
  report("5. Default integration time readback",
         intTime >= 2 && intTime <= 1000);

  // Test 6: Default sharpener
  uint8_t sharp = vl53l5cx.getSharpenerPercent();
  Serial.print(F("   Sharpener: "));
  Serial.print(sharp);
  Serial.println(F("%"));
  report("6. Default sharpener readback", sharp != 0xFF);

  // Test 7: Start ranging
  bool startOk = vl53l5cx.startRanging();
  report("7. startRanging() succeeds", startOk);

  // Test 8: Data ready within 5 seconds
  bool dataReady = false;
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (vl53l5cx.isDataReady()) {
      dataReady = true;
      break;
    }
    delay(5);
  }
  Serial.print(F("   Data ready after "));
  Serial.print(millis() - start);
  Serial.println(F(" ms"));
  report("8. Data ready within 5s", dataReady);

  // Test 9 & 10: Read data
  if (dataReady) {
    VL53L5CX_ResultsData results;
    bool readOk = vl53l5cx.getRangingData(&results);
    report("9. getRangingData() succeeds", readOk);

    if (readOk) {
      // Check distances are sane
      uint8_t zones = (res == 16) ? 16 : 64;
      uint16_t nonZero = 0;
      uint16_t validRange = 0;
      for (uint8_t i = 0; i < zones; i++) {
        if (results.distance_mm[i] != 0)
          nonZero++;
        if (results.distance_mm[i] > 0 && results.distance_mm[i] < 4000)
          validRange++;
      }
      Serial.print(F("   Non-zero distances: "));
      Serial.print(nonZero);
      Serial.print(F("/"));
      Serial.println(zones);
      Serial.print(F("   Valid range (1-4000mm): "));
      Serial.print(validRange);
      Serial.print(F("/"));
      Serial.println(zones);
      report("10. Distance values sane", nonZero > 0 && validRange > 0);
    } else {
      report("10. Distance values sane", false);
    }
  } else {
    report("9. getRangingData() succeeds", false);
    report("10. Distance values sane", false);
  }

  vl53l5cx.stopRanging();

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
