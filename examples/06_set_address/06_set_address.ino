/*!
 * @file vl53l5cx_set_address.ino
 *
 * Example demonstrating how to change the I2C address of the VL53L5CX sensor
 *
 * This is useful when using multiple sensors on the same I2C bus.
 * The new address is stored in RAM, so it resets to default (0x29)
 * on power cycle. To use multiple sensors, change addresses at startup
 * using the LPn pin to selectively enable each sensor.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_VL53L5CX.h>

Adafruit_VL53L5CX vl53l5cx;

#define DEFAULT_ADDRESS 0x29
#define NEW_ADDRESS 0x30

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("Adafruit VL53L5CX - Set Address Example"));
  Serial.println(F("========================================"));
  Serial.println();

  Serial.print(F("Initializing sensor at default address 0x"));
  Serial.print(DEFAULT_ADDRESS, HEX);
  Serial.println(F("..."));
  Serial.println(F("(this can take up to 10 seconds)"));

  // Initialize with default address, Wire bus, 400kHz clock
  if (!vl53l5cx.begin(DEFAULT_ADDRESS, &Wire, 400000)) {
    halt(F("Failed to initialize VL53L5CX sensor!"));
  }

  Serial.println(F("Sensor initialized!"));
  Serial.println();

  // Show current address
  Serial.print(F("Current I2C address: 0x"));
  Serial.println(DEFAULT_ADDRESS, HEX);
  Serial.println();

  Serial.println(F("Send any character to change address to 0x30..."));

  // Wait for user input
  while (!Serial.available()) {
    delay(10);
  }
  // Clear the input buffer
  while (Serial.available()) {
    Serial.read();
  }

  // Change the address
  Serial.print(F("Changing address to 0x"));
  Serial.print(NEW_ADDRESS, HEX);
  Serial.println(F("..."));

  if (!vl53l5cx.setAddress(NEW_ADDRESS)) {
    halt(F("Failed to change address!"));
  }

  Serial.println(F("Address changed successfully!"));
  Serial.println();

  // Verify by starting ranging at the new address
  Serial.println(F("Verifying sensor responds at new address..."));

  if (!vl53l5cx.startRanging()) {
    halt(F("Failed to start ranging at new address!"));
  }

  Serial.println(F("SUCCESS! Sensor is responding at new address."));
  Serial.println();

  // Scan I2C bus to show the change
  Serial.println(F("I2C scan showing device at new address:"));
  scanI2C();

  Serial.println();
  Serial.println(F("NOTE: The address change is stored in RAM only."));
  Serial.println(F("The sensor will return to default address (0x29)"));
  Serial.println(F("after a power cycle."));
  Serial.println();
  Serial.println(F("To use multiple sensors:"));
  Serial.println(F("1. Connect LPn pins of all sensors to GPIO pins"));
  Serial.println(F("2. Hold all LPn LOW to disable all sensors"));
  Serial.println(F("3. Set one LPn HIGH, change that sensor's address"));
  Serial.println(F("4. Repeat for each sensor with unique addresses"));
}

void loop() {
  // Nothing to do here
  delay(1000);
}

void scanI2C() {
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  Found device at 0x"));
      if (addr < 16)
        Serial.print(F("0"));
      Serial.println(addr, HEX);
    }
  }
}

void halt(const __FlashStringHelper* msg) {
  Serial.println(msg);
  while (1)
    delay(10);
}
