/*!
 * @file hw_test_14_i2c_rst.ino
 *
 * Hardware test: VL53L5CX I2C_RST pin
 *
 * Verifies that toggling I2C_RST recovers the sensor's I2C interface
 * after intentionally wedging the bus with a partial transfer.
 *
 * Tests:
 *  1. Sensor responds on I2C at startup
 *  2. begin() and ranging work normally
 *  3. Wedge the I2C bus with a partial bit-bang transfer
 *  4. Sensor does NOT respond (bus is stuck)
 *  5. Toggle I2C_RST to recover
 *  6. Sensor responds on I2C after reset
 *  7. begin() succeeds after I2C reset
 *  8. Ranging works after I2C reset (firmware intact)
 *
 * Wiring: VL53L5CX I2C_RST -> GPIO 13
 *         VL53L5CX via STEMMA QT / I2C
 */

#include <Adafruit_VL53L5CX.h>

#define I2C_RST_PIN 13

// Default I2C pins — adjust for your board
#ifdef ESP32
#define MY_SDA 23
#define MY_SCL 22
#else
// RP2040 Feather default
#define MY_SDA 24
#define MY_SCL 25
#endif

Adafruit_VL53L5CX vl53l5cx;

uint8_t passed = 0;
uint8_t failed = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW Test 14: I2C_RST Pin ==="));
  Serial.println();

  // I2C_RST idles LOW (47k pulldown on breakout)
  pinMode(I2C_RST_PIN, OUTPUT);
  digitalWrite(I2C_RST_PIN, LOW);

  Wire.begin();
  delay(100);

  // Test 1: Sensor on I2C
  bool found = i2cPresent(0x29);
  Serial.print(F("   I2C scan: "));
  Serial.println(found ? F("found 0x29") : F("not found!"));
  report("1. Sensor on I2C at startup", found);

  // Test 2: Normal init and ranging
  bool initOk = vl53l5cx.begin(0x29, &Wire, 400000);
  report("2. begin() succeeds", initOk);
  if (!initOk) {
    Serial.println(F("   Cannot continue without init."));
    printSummary();
    while (1)
      delay(10);
  }

  vl53l5cx.setRangingFrequency(15);
  vl53l5cx.startRanging();
  VL53L5CX_ResultsData results;
  bool gotData = waitAndRead(&results);
  vl53l5cx.stopRanging();
  delay(100);
  if (gotData) {
    Serial.print(F("   Distance[0]: "));
    Serial.print(results.distance_mm[0]);
    Serial.println(F(" mm"));
  }
  report("2b. Ranging works", gotData);

  // Test 3: Wedge the I2C bus
  Serial.println();
  Serial.println(F("   --- Wedging I2C bus ---"));
  wedgeI2CBus();
  delay(10);

  // Test 4: Sensor should NOT respond (bus is stuck)
  // Re-init Wire since we bit-banged the pins
  Wire.begin();
  delay(10);
  bool stuckCheck = i2cPresent(0x29);
  Serial.print(F("   I2C scan after wedge: "));
  Serial.println(stuckCheck ? F("still responds (wedge failed)")
                            : F("no response (bus stuck!)"));
  report("3. Bus is wedged (no I2C response)", !stuckCheck);

  if (stuckCheck) {
    Serial.println(F("   Wedge didn't work — sensor still responds."));
    Serial.println(F("   Skipping recovery tests."));
    // Still test toggle doesn't break things
  }

  // Test 5: Toggle I2C_RST to recover
  Serial.println();
  Serial.println(F("   --- Toggling I2C_RST ---"));
  digitalWrite(I2C_RST_PIN, HIGH);
  delay(2); // 2ms pulse — conservative
  digitalWrite(I2C_RST_PIN, LOW);
  delay(10); // let sensor I2C settle

  // Re-init Wire after the reset
  Wire.begin();
  delay(10);

  // Test 6: Sensor should respond again
  bool recovered = i2cPresent(0x29);
  Serial.print(F("   I2C scan after reset: "));
  Serial.println(recovered ? F("found 0x29!") : F("not found"));
  report("4. Sensor responds after I2C_RST", recovered);

  // Test 7: begin() after recovery
  bool reinitOk = vl53l5cx.begin(0x29, &Wire, 400000);
  report("5. begin() after I2C_RST", reinitOk);

  // Test 8: Ranging still works (firmware wasn't wiped)
  if (reinitOk) {
    vl53l5cx.setRangingFrequency(15);
    vl53l5cx.startRanging();
    bool gotData2 = waitAndRead(&results);
    vl53l5cx.stopRanging();
    if (gotData2) {
      Serial.print(F("   Distance[0]: "));
      Serial.print(results.distance_mm[0]);
      Serial.println(F(" mm"));
    }
    report("6. Ranging works after I2C_RST", gotData2);
  } else {
    report("6. Ranging works after I2C_RST", false);
  }

  // Cleanup
  digitalWrite(I2C_RST_PIN, LOW);
  pinMode(I2C_RST_PIN, INPUT);

  printSummary();
}

void loop() {
  delay(1000);
}

/*!
 * @brief Wedge the I2C bus by bit-banging a partial transfer
 *
 * Strategy: take over SDA/SCL as GPIO, send a START condition
 * followed by a few address bits but NOT a complete byte.
 * The sensor's I2C peripheral is now stuck mid-byte, waiting
 * for more clocks. SDA may be held low by the sensor.
 */
void wedgeI2CBus() {
  // Shut down Wire so we can bit-bang
  Wire.end();
  delay(5);

  pinMode(MY_SDA, OUTPUT);
  pinMode(MY_SCL, OUTPUT);

  // Bus idle: both HIGH
  digitalWrite(MY_SDA, HIGH);
  digitalWrite(MY_SCL, HIGH);
  delayMicroseconds(10);

  // START condition: SDA falls while SCL is HIGH
  digitalWrite(MY_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(MY_SCL, LOW);
  delayMicroseconds(10);

  // Clock out 4 bits of the address byte (0x29 write = 0x52 = 01010010)
  // Bit 7: 0
  digitalWrite(MY_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(MY_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(MY_SCL, LOW);
  delayMicroseconds(5);

  // Bit 6: 1
  digitalWrite(MY_SDA, HIGH);
  delayMicroseconds(5);
  digitalWrite(MY_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(MY_SCL, LOW);
  delayMicroseconds(5);

  // Bit 5: 0
  digitalWrite(MY_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(MY_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(MY_SCL, LOW);
  delayMicroseconds(5);

  // Bit 4: 1
  digitalWrite(MY_SDA, HIGH);
  delayMicroseconds(5);
  digitalWrite(MY_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(MY_SCL, LOW);
  delayMicroseconds(5);

  // STOP HERE — leave bus mid-byte!
  // Release SDA as input so sensor can drive it
  // Leave SCL LOW — sensor is waiting for more clocks
  pinMode(MY_SDA, INPUT);
  // Don't send any more clocks, don't send STOP

  Serial.println(F("   Sent START + 4 bits, abandoned transfer."));
  Serial.print(F("   SDA state: "));
  Serial.println(digitalRead(MY_SDA) ? F("HIGH") : F("LOW (sensor holding!)"));
  Serial.print(F("   SCL state: LOW (we left it)"));
  Serial.println();

  // Release SCL too — if sensor is stretching, SDA stays stuck
  pinMode(MY_SCL, INPUT);
  delayMicroseconds(100);

  Serial.print(F("   After SCL release — SDA: "));
  Serial.print(digitalRead(MY_SDA) ? F("HIGH") : F("LOW"));
  Serial.print(F(", SCL: "));
  Serial.println(digitalRead(MY_SCL) ? F("HIGH") : F("LOW"));
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

void printSummary() {
  Serial.println();
  Serial.println(F("=== SUMMARY ==="));
  Serial.print(passed);
  Serial.print(F(" passed, "));
  Serial.print(failed);
  Serial.println(F(" failed"));
  Serial.print(F("Result: "));
  Serial.println(failed == 0 ? F("ALL PASSED") : F("SOME FAILED"));
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
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
