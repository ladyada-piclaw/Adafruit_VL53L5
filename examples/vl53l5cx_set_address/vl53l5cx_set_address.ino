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
#include <Wire.h>

Adafruit_VL53L5CX vl53l5cx;

#define DEFAULT_ADDRESS 0x29
#define NEW_ADDRESS 0x30

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit VL53L5CX - Set Address Example");
  Serial.println("========================================");
  Serial.println();

  Serial.print("Initializing sensor at default address 0x");
  Serial.print(DEFAULT_ADDRESS, HEX);
  Serial.println("...");
  Serial.println("(this can take up to 10 seconds)");

  if (!vl53l5cx.begin(DEFAULT_ADDRESS)) {
    Serial.println("Failed to initialize VL53L5CX sensor!");
    Serial.println("Check wiring and try again.");
    while (1)
      delay(10);
  }

  Serial.println("Sensor initialized!");
  Serial.println();

  // Show current address
  Serial.print("Current I2C address: 0x");
  Serial.println(DEFAULT_ADDRESS, HEX);
  Serial.println();

  Serial.println("Send any character to change address to 0x30...");

  // Wait for user input
  while (!Serial.available()) {
    delay(10);
  }
  // Clear the input buffer
  while (Serial.available()) {
    Serial.read();
  }

  // Change the address
  Serial.print("Changing address to 0x");
  Serial.print(NEW_ADDRESS, HEX);
  Serial.println("...");

  if (!vl53l5cx.setAddress(NEW_ADDRESS)) {
    Serial.println("Failed to change address!");
    while (1)
      delay(10);
  }

  Serial.println("Address changed successfully!");
  Serial.println();

  // Verify by starting ranging at the new address
  Serial.println("Verifying sensor responds at new address...");

  if (!vl53l5cx.startRanging()) {
    Serial.println("Failed to start ranging at new address!");
    while (1)
      delay(10);
  }

  Serial.println("SUCCESS! Sensor is responding at new address.");
  Serial.println();

  // Scan I2C bus to show the change
  Serial.println("I2C scan showing device at new address:");
  scanI2C();

  Serial.println();
  Serial.println("NOTE: The address change is stored in RAM only.");
  Serial.println("The sensor will return to default address (0x29)");
  Serial.println("after a power cycle.");
  Serial.println();
  Serial.println("To use multiple sensors:");
  Serial.println("1. Connect LPn pins of all sensors to GPIO pins");
  Serial.println("2. Hold all LPn LOW to disable all sensors");
  Serial.println("3. Set one LPn HIGH, change that sensor's address");
  Serial.println("4. Repeat for each sensor with unique addresses");
}

void loop() {
  // Nothing to do here
  delay(1000);
}

void scanI2C() {
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
}
