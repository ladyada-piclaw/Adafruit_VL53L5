/*!
 * @file 03_detection_threshold.ino
 *
 * Example demonstrating detection thresholds on the VL53L5CX
 *
 * Detection thresholds allow the sensor to automatically trigger an
 * interrupt when certain conditions are met (e.g., object closer than
 * a specified distance). This is more efficient than polling all zones
 * in software.
 *
 * This example configures all 16 zones (4x4 mode) to trigger when an
 * object is detected closer than 500mm. The INT pin goes LOW when any
 * zone triggers, allowing wake-on-detection applications.
 *
 * Wiring:
 *   - Connect VL53L5CX via STEMMA QT / I2C
 *   - Connect VL53L5CX INT pin to INT_PIN (default A2)
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_VL53L5CX.h>

// INT pin - connect to VL53L5CX INT output
// Active-low, open-drain - use internal pullup
#ifdef ESP32
#define INT_PIN 14
#else
#define INT_PIN A2
#endif

// Detection threshold distance in mm
#define THRESHOLD_DISTANCE_MM 500

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("VL53L5CX Detection Threshold Example"));
  Serial.println(F("====================================="));
  Serial.println();

  // Configure INT pin with internal pullup (sensor output is open-drain)
  pinMode(INT_PIN, INPUT_PULLUP);

  Serial.println(F("Initializing sensor... (this can take up to 10 seconds)"));

  if (!vl53l5cx.begin()) {
    Serial.println(F("Failed to initialize VL53L5CX sensor!"));
    while (1)
      delay(10);
  }

  Serial.println(F("Sensor initialized!"));

  // Use 4x4 mode (16 zones) - simpler for threshold demo
  if (!vl53l5cx.setResolution(16)) {
    Serial.println(F("Failed to set resolution!"));
  }

  // Set 30 Hz ranging frequency for snappy detection
  if (!vl53l5cx.setRangingFrequency(30)) {
    Serial.println(F("Failed to set ranging frequency!"));
  }

  // IMPORTANT: Stop ranging before configuring thresholds
  vl53l5cx.stopRanging();

  // Configure detection thresholds for all 16 zones
  VL53L5CX_DetectionThresholds thresholds[VL53L5CX_NB_THRESHOLDS];
  memset(thresholds, 0, sizeof(thresholds));

  Serial.println(F("Configuring detection thresholds..."));

  for (uint8_t zone = 0; zone < 16; zone++) {
    thresholds[zone].zone_num = zone;
    thresholds[zone].measurement = VL53L5CX_DISTANCE_MM;
    thresholds[zone].type = VL53L5CX_LESS_THAN_EQUAL_MIN_CHECKER;
    thresholds[zone].param_low_thresh = THRESHOLD_DISTANCE_MM;
    thresholds[zone].param_high_thresh = THRESHOLD_DISTANCE_MM;
    thresholds[zone].mathematic_operation = VL53L5CX_OPERATION_OR;
  }

  // Mark end of threshold list
  thresholds[16].zone_num = VL53L5CX_LAST_THRESHOLD;

  if (!vl53l5cx.setDetectionThresholds(thresholds)) {
    Serial.println(F("Failed to set detection thresholds!"));
  }

  if (!vl53l5cx.setDetectionThresholdsEnable(true)) {
    Serial.println(F("Failed to enable detection thresholds!"));
  }

  Serial.println(F("Detection thresholds configured and enabled!"));

  // Start ranging with thresholds active
  if (!vl53l5cx.startRanging()) {
    Serial.println(F("Failed to start ranging!"));
    while (1)
      delay(10);
  }

  Serial.println();
  Serial.print(F("Detection threshold: "));
  Serial.print(THRESHOLD_DISTANCE_MM);
  Serial.println(F("mm"));
  Serial.println(F("INT pin will go LOW when any zone detects an object"));
  Serial.println(F("closer than the threshold distance."));
  Serial.println();
}

void loop() {
  // Check if INT pin fired (active LOW)
  if (digitalRead(INT_PIN) == LOW) {
    // Threshold triggered - read the data
    if (vl53l5cx.getRangingData(&results)) {
      Serial.print(F("Object detected in zones: "));

      bool first = true;
      uint8_t triggerCount = 0;

      for (uint8_t zone = 0; zone < 16; zone++) {
        int16_t distance = results.distance_mm[zone];

        if (distance > 0 && distance <= THRESHOLD_DISTANCE_MM) {
          if (!first) {
            Serial.print(F(", "));
          }
          Serial.print(zone);
          Serial.print(F(" ("));
          Serial.print(distance);
          Serial.print(F("mm)"));
          first = false;
          triggerCount++;
        }
      }

      if (triggerCount == 0) {
        Serial.print(F("(none - false trigger or object moved)"));
      }

      Serial.println();
    }
  }

  delay(5);
}
