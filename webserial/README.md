# VL53L5CX WebSerial Demo

A browser-based visualization for the Adafruit VL53L5CX 8x8 Time-of-Flight sensor.

## Requirements

- Chrome or Edge browser (Web Serial API support)
- Microcontroller with VL53L5CX sensor running the `vl53l5cx_webserial` example sketch

## Usage

1. Upload `examples/vl53l5cx_webserial/vl53l5cx_webserial.ino` to your microcontroller
2. Open `index.html` in Chrome/Edge (or visit the hosted version)
3. Click "Connect" and select your serial port
4. Watch the live distance grid visualization!

## Features

- **Live visualization**: NxN grid showing distances with color-coded cells
  - Red/warm = close (0mm)
  - Blue/purple = far (2000mm+)
  - Gray = invalid/no data
- **Adjustable settings**:
  - Update rate: 1, 2, 5, 10, 15, 30 Hz
  - Resolution: 4×4 (16 zones) or 8×8 (64 zones)
- **Serial log**: Collapsible raw serial data view

## Serial Protocol

The sketch outputs frames in this format:
```
FRAME_START
RES:64
RATE:15
D:123,456,789,...
S:5,5,5,...
FRAME_END
```

Commands accepted:
- `RATE:X` — set ranging frequency (1-60 Hz)
- `RES:16` or `RES:64` — set resolution

## Hosted Version

This demo is hosted at:
https://adafruit.github.io/Adafruit_VL53L5CX/webserial/

## License

MIT License - Adafruit Industries
