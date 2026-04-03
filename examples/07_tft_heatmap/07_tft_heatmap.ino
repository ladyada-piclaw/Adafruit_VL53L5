/*
 * MIT License
 *
 * Copyright (c) 2026 Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * @file 07_tft_heatmap.ino
 *
 * TFT heatmap demo for Adafruit VL53L5CX 8x8 ToF sensor
 *
 * Displays an 8x8 distance heatmap on the Adafruit Feather ESP32-S2 TFT.
 * Press BOOT button to toggle bicubic interpolation (8x8 <-> 16x16).
 * Connect the sensor via STEMMA QT/I2C.
 *
 * Written by Limor ladyada Fried with assistance from Claude Code
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_VL53L5CX.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <SPI.h>

#define BOOT_PIN 0 // BOOT button on ESP32-S2

Adafruit_VL53L5CX vl53l5cx;
VL53L5CX_ResultsData results;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

const uint16_t SCREEN_W = 240;
const uint16_t SCREEN_H = 135;
const uint16_t INVALID_COLOR = 0x4208; // dark grey
const int16_t MIN_DIST_MM = 0;
const int16_t MAX_DIST_MM = 2000;

bool useInterpolation = true;
bool lastButtonState = HIGH;

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t hsvToRgb565(float h, float s, float v) {
  if (s <= 0) {
    uint8_t c = (uint8_t)(v * 255.0 + 0.5);
    return color565(c, c, c);
  }

  while (h < 0) {
    h += 360.0;
  }
  while (h >= 360.0) {
    h -= 360.0;
  }

  float hh = h / 60.0;
  int sector = (int)hh;
  float f = hh - sector;
  float p = v * (1.0 - s);
  float q = v * (1.0 - s * f);
  float t = v * (1.0 - s * (1.0 - f));

  float r = 0;
  float g = 0;
  float b = 0;

  switch (sector) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    default:
      r = v;
      g = p;
      b = q;
      break;
  }

  uint8_t rr = (uint8_t)(r * 255.0 + 0.5);
  uint8_t gg = (uint8_t)(g * 255.0 + 0.5);
  uint8_t bb = (uint8_t)(b * 255.0 + 0.5);
  return color565(rr, gg, bb);
}

uint16_t distanceToColor(int16_t distance_mm) {
  if (distance_mm < MIN_DIST_MM) {
    distance_mm = MIN_DIST_MM;
  }
  if (distance_mm > MAX_DIST_MM) {
    distance_mm = MAX_DIST_MM;
  }

  float t =
      (float)(distance_mm - MIN_DIST_MM) / (float)(MAX_DIST_MM - MIN_DIST_MM);
  float hue = t * 240.0; // red -> yellow -> green -> blue
  return hsvToRgb565(hue, 1.0, 1.0);
}

// Bicubic interpolation helper
float cubicInterp(float p0, float p1, float p2, float p3, float t) {
  return p1 + 0.5 * t *
                  (p2 - p0 +
                   t * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 +
                        t * (3.0 * (p1 - p2) + p3 - p0)));
}

// Get distance value with clamped boundary (mirror edges)
float getVal(float grid[8][8], int r, int c) {
  r = constrain(r, 0, 7);
  c = constrain(c, 0, 7);
  return grid[r][c];
}

// Bicubic interpolate 8x8 -> 16x16
void bicubicInterpolate(float src[8][8], float dst[16][16]) {
  for (int dr = 0; dr < 16; dr++) {
    for (int dc = 0; dc < 16; dc++) {
      float srcR = (dr + 0.5) * 8.0 / 16.0 - 0.5;
      float srcC = (dc + 0.5) * 8.0 / 16.0 - 0.5;
      int r0 = (int)floor(srcR);
      int c0 = (int)floor(srcC);
      float tr = srcR - r0;
      float tc = srcC - c0;

      // Interpolate 4 rows, then across columns
      float cols[4];
      for (int i = -1; i <= 2; i++) {
        cols[i + 1] = cubicInterp(
            getVal(src, r0 + i, c0 - 1), getVal(src, r0 + i, c0),
            getVal(src, r0 + i, c0 + 1), getVal(src, r0 + i, c0 + 2), tc);
      }
      float val = cubicInterp(cols[0], cols[1], cols[2], cols[3], tr);
      dst[dr][dc] = constrain(val, MIN_DIST_MM, MAX_DIST_MM);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // give USB serial a moment but don't block

  Serial.println(F("Adafruit VL53L5CX TFT Heatmap Demo"));
  Serial.println(F("=================================="));
  Serial.println(
      F("Press BOOT button to toggle interpolation (8x8 <-> 16x16)"));
  Serial.println(F("Initializing sensor... (this can take up to 10 seconds)"));

  pinMode(BOOT_PIN, INPUT_PULLUP);

  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  tft.init(135, 240);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  // Initialize with I2C address 0x29, Wire bus, 400kHz clock
  if (!vl53l5cx.begin(VL53L5CX_DEFAULT_ADDRESS, &Wire, 400000)) {
    halt(F("Failed to initialize VL53L5CX sensor!"));
  }

  Serial.println(F("Sensor initialized!"));

  if (!vl53l5cx.setResolution(64)) {
    halt(F("Failed to set resolution!"));
  }

  if (!vl53l5cx.setRangingFrequency(15)) {
    halt(F("Failed to set ranging frequency!"));
  }

  if (!vl53l5cx.startRanging()) {
    halt(F("Failed to start ranging!"));
  }

  Serial.print(F("Resolution: "));
  Serial.println(vl53l5cx.getResolution());
  Serial.print(F("Ranging frequency: "));
  Serial.print(vl53l5cx.getRangingFrequency());
  Serial.println(F(" Hz"));
}

void drawFrame() {
  const uint16_t bg = ST77XX_BLACK;
  const uint16_t fg = ST77XX_WHITE;

  int16_t gridDim = useInterpolation ? 16 : 8;
  int16_t cellSize = SCREEN_H / gridDim;
  int16_t gridSize = cellSize * gridDim;
  const int16_t gridX = 4;
  const int16_t gridY = (SCREEN_H - gridSize) / 2;
  const int16_t panelX = gridX + gridSize + 6;

  canvas.fillScreen(bg);

  // --- Build 8x8 grid + find min/max ---
  // The raw 8x8 non-interp version draws correctly with this orientation,
  // so we store into raw[r][c] the same way for both paths.
  uint16_t minDist = MAX_DIST_MM;
  uint16_t maxDist = 0;
  bool hasValid = false;
  float raw[8][8];
  bool valid[8][8];

  // Store raw data indexed by [row][col] from ST driver zone ordering
  const uint8_t width = 8;
  for (int x = width - 1; x >= 0; x--) {
    for (int y = width * (width - 1); y >= 0; y -= width) {
      int idx = x + y;
      int row = y / width;
      int col = x;
      int16_t dist = results.distance_mm[idx];
      uint8_t status = results.target_status[idx];

      if (status == 5 || status == 9) {
        raw[row][col] = dist;
        valid[row][col] = true;
        if (dist < minDist)
          minDist = dist;
        if (dist > maxDist)
          maxDist = dist;
        hasValid = true;
      } else {
        raw[row][col] = -1;
        valid[row][col] = false;
      }
    }
  }

  // Fill invalid zones for interpolation
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      if (!valid[r][c]) {
        raw[r][c] = hasValid ? (minDist + maxDist) / 2.0 : 0;
      }
    }
  }

  if (useInterpolation) {
    // Bicubic interpolate 8x8 -> 16x16
    float interp[16][16];
    bicubicInterpolate(raw, interp);

    for (int r = 0; r < 16; r++) {
      for (int c = 0; c < 16; c++) {
        uint16_t color = distanceToColor((int16_t)interp[r][c]);
        // Swap X/Y and flip: col mapped from row, row mapped from col
        int16_t px = gridX + (15 - r) * cellSize;
        int16_t py = gridY + (15 - c) * cellSize;
        canvas.fillRect(px, py, cellSize, cellSize, color);
      }
    }
  } else {
    // Draw raw 8x8
    for (int r = 0; r < 8; r++) {
      for (int c = 0; c < 8; c++) {
        uint16_t color =
            valid[r][c] ? distanceToColor((int16_t)raw[r][c]) : INVALID_COLOR;
        // Swap X/Y and flip: col mapped from row, row mapped from col
        int16_t px = gridX + (7 - r) * cellSize;
        int16_t py = gridY + (7 - c) * cellSize;
        canvas.fillRect(px, py, cellSize - 1, cellSize - 1, color);
      }
    }
  }

  // --- Right panel: title + stats ---
  canvas.setFont(&FreeSansBold9pt7b);
  canvas.setTextColor(fg, bg);

  canvas.setCursor(panelX, 16);
  canvas.print(F("Adafruit"));

  canvas.setCursor(panelX, 38);
  canvas.print(F("VL53L5CX"));

  // Min/Max
  canvas.setCursor(panelX, 72);
  canvas.print(F("Min "));
  if (hasValid) {
    canvas.print(minDist);
  } else {
    canvas.print(F("--"));
  }

  canvas.setCursor(panelX, 96);
  canvas.print(F("Max "));
  if (hasValid) {
    canvas.print(maxDist);
  } else {
    canvas.print(F("--"));
  }

  // Mode indicator
  canvas.setFont(&FreeSansBold9pt7b);
  canvas.setTextColor(0x8410, bg);
  canvas.setCursor(panelX, 120);
  canvas.print(useInterpolation ? F("16x16") : F("8x8"));

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), SCREEN_W, SCREEN_H);
}

void loop() {
  // Check BOOT button for toggle
  bool buttonState = digitalRead(BOOT_PIN);
  if (buttonState == LOW && lastButtonState == HIGH) {
    useInterpolation = !useInterpolation;
    Serial.print(F("Interpolation: "));
    Serial.println(useInterpolation ? F("ON (16x16)") : F("OFF (8x8)"));
  }
  lastButtonState = buttonState;

  if (vl53l5cx.isDataReady()) {
    if (vl53l5cx.getRangingData(&results)) {
      drawFrame();
    }
  }

  delay(5);
}

void halt(const __FlashStringHelper* msg) {
  Serial.println(msg);
  while (1)
    delay(10);
}
