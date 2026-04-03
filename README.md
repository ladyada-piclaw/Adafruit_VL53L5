# Adafruit VL53L5CX Library
[![Arduino Library CI](https://github.com/adafruit/Adafruit_VL53L5/actions/workflows/githubci.yml/badge.svg)](https://github.com/adafruit/Adafruit_VL53L5/actions/workflows/githubci.yml)

Arduino library for the ST VL53L5CX 8x8 multizone Time-of-Flight sensor. Supports up to 4m range with 4x4 or 8x8 resolution.

**Features**
- Resolution control (4x4 or 8x8)
- Ranging frequency selection
- Sharpener control
- Target order selection
- Ranging mode selection
- Power mode control
- Detection thresholds
- Motion indicator
- Crosstalk (xtalk) calibration
- I2C address change
- LPn pin support

**Hardware Requirements**
- Requires an ARM-class MCU (ESP32, SAMD, RP2040, nRF52840, etc.)
- Firmware upload on power-on requires ~84KB

**Dependencies**
- Adafruit BusIO

**Examples**
- `examples/01_simpletest` - Basic ranging readout
- `examples/02_ascii_art` - Print an 8x8 distance grid as ASCII art
- `examples/03_detection_threshold` - Configure detection thresholds and report triggers
- `examples/04_motion_detect` - Read motion indicator output
- `examples/05_webserial` - Stream data for the browser heatmap viewer
- `examples/06_set_address` - Change the I2C address
- `examples/07_tft_heatmap` - Display a heatmap on a TFT

**WebSerial**
The `webserial/` directory contains a browser-based heatmap visualization that pairs with the WebSerial example.

**License**
MIT

**Product Page**
https://www.adafruit.com/product/XXXXX
