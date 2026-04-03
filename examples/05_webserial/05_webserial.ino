/*!
 * @file vl53l5cx_webserial.ino
 *
 * Web Serial demo for the Adafruit VL53L5CX 8x8 ToF sensor
 *
 * Outputs structured data for the WebSerial visualization page.
 * Accepts serial commands to change resolution and ranging frequency.
 *
 * Output format (one frame per block):
 *   FRAME_START
 *   RES:64
 *   RATE:15
 *   D:123,456,789,...  (comma-separated distance_mm values)
 *   S:12,34,56,...     (comma-separated status values)
 *   FRAME_END
 *
 * Commands:
 *   RATE:1 through RATE:60 — set ranging frequency in Hz
 *   RES:16 or RES:64 — set resolution (4x4 or 8x8)
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude/piclaw
 */

#include <Adafruit_VL53L5CX.h>

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;

uint8_t currentResolution = 64;
uint8_t currentRate = 15;
String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("VL53L5CX WebSerial Demo"));
  Serial.println(F("======================="));
  Serial.println(F("Initializing sensor... (this can take up to 10 seconds)"));

  // Initialize with I2C address 0x29, Wire bus, 400kHz clock
  if (!vl53l5cx.begin(VL53L5CX_DEFAULT_ADDRESS, &Wire, 400000)) {
    halt(F("ERROR: Failed to initialize VL53L5CX sensor!"));
  }

  Serial.println(F("Sensor initialized!"));

  // Set default 8x8 resolution (64 zones)
  if (!vl53l5cx.setResolution(currentResolution)) {
    halt(F("ERROR: Failed to set resolution!"));
  }

  // Set default ranging frequency
  if (!vl53l5cx.setRangingFrequency(currentRate)) {
    halt(F("ERROR: Failed to set ranging frequency!"));
  }

  // Start ranging
  if (!vl53l5cx.startRanging()) {
    halt(F("ERROR: Failed to start ranging!"));
  }

  Serial.print(F("Resolution: "));
  Serial.println(currentResolution);
  Serial.print(F("Rate: "));
  Serial.print(currentRate);
  Serial.println(F(" Hz"));
  Serial.println(F("READY"));
}

void loop() {
  // Check for incoming serial commands
  processSerialInput();

  // Check for new ranging data
  if (vl53l5cx.isDataReady()) {
    if (vl53l5cx.getRangingData(&results)) {
      outputFrame();
    }
  }

  delay(5); // Small delay between polling
}

/**************************************************************************/
/*!
    @brief  Process incoming serial commands
*/
/**************************************************************************/
void processSerialInput(void) {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

/**************************************************************************/
/*!
    @brief  Handle a parsed command string
    @param  cmd The command string (e.g., "RATE:15" or "RES:64")
*/
/**************************************************************************/
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith(F("RATE:"))) {
    int rate = cmd.substring(5).toInt();
    if (rate >= 1 && rate <= 60) {
      // Stop ranging before changing settings
      vl53l5cx.stopRanging();

      if (vl53l5cx.setRangingFrequency(rate)) {
        currentRate = rate;
        Serial.print(F("OK RATE:"));
        Serial.println(currentRate);
      } else {
        Serial.println(F("ERROR: Failed to set rate"));
      }

      vl53l5cx.startRanging();
    } else {
      Serial.println(F("ERROR: Rate must be 1-60"));
    }
  } else if (cmd.startsWith(F("RES:"))) {
    int res = cmd.substring(4).toInt();
    if (res == 16 || res == 64) {
      // Stop ranging before changing resolution
      vl53l5cx.stopRanging();

      if (vl53l5cx.setResolution(res)) {
        currentResolution = res;
        Serial.print(F("OK RES:"));
        Serial.println(currentResolution);
      } else {
        Serial.println(F("ERROR: Failed to set resolution"));
      }

      vl53l5cx.startRanging();
    } else {
      Serial.println(F("ERROR: Resolution must be 16 or 64"));
    }
  } else {
    Serial.print(F("ERROR: Unknown command: "));
    Serial.println(cmd);
  }
}

/**************************************************************************/
/*!
    @brief  Output a complete frame of ranging data
*/
/**************************************************************************/
void outputFrame(void) {
  Serial.println(F("FRAME_START"));

  Serial.print(F("RES:"));
  Serial.println(currentResolution);

  Serial.print(F("RATE:"));
  Serial.println(currentRate);

  // Output distances (reordered to match physical sensor orientation)
  uint8_t width = (currentResolution == 16) ? 4 : 8;
  Serial.print(F("D:"));
  {
    bool first = true;
    for (int x = width - 1; x >= 0; x--) {
      for (int y = width * (width - 1); y >= 0; y -= width) {
        if (!first)
          Serial.print(F(","));
        Serial.print(results.distance_mm[x + y]);
        first = false;
      }
    }
    Serial.println();
  }

  // Output status (same reordering)
  Serial.print(F("S:"));
  {
    bool first = true;
    for (int x = width - 1; x >= 0; x--) {
      for (int y = width * (width - 1); y >= 0; y -= width) {
        if (!first)
          Serial.print(F(","));
        Serial.print(results.target_status[x + y]);
        first = false;
      }
    }
    Serial.println();
  }

  Serial.println(F("FRAME_END"));
}

void halt(const __FlashStringHelper* msg) {
  Serial.println(msg);
  while (1)
    delay(10);
}
