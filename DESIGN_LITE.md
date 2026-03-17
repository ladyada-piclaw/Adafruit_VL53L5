# VL53L5CX Lightweight BusIO Driver — Design Notes

## Why
The ST ULD (Ultra Lite Driver) works but has drawbacks:
- 86KB firmware blob in flash (const arrays)
- ~2.7KB RAM for Configuration struct (offset/xtalk/temp buffers)
- Complex init sequence with opaque magic numbers
- C-style API that doesn't integrate cleanly with Arduino patterns

A lightweight BusIO-native driver would:
- Still need the firmware blob (the sensor requires it — no way around this)
- Eliminate the ST ULD C code entirely
- Use Adafruit_BusIO_Register for all register/DCI access
- Reduce RAM by not pre-allocating large offset/xtalk/temp buffers
- Be portable and readable

## Architecture Analysis

### What MUST stay from ST
1. **Firmware blob** (`VL53L5CX_FIRMWARE[]`, 86KB) — sensor has no internal ROM, firmware is uploaded every power-on
2. **Default configuration** (`VL53L5CX_DEFAULT_CONFIGURATION[]`, 971 bytes) — sent after firmware boot
3. **Default xtalk** (`VL53L5CX_DEFAULT_XTALK[]`, 776 bytes) — factory calibration defaults
4. **NVM command** (`VL53L5CX_GET_NVM_CMD[]`, 40 bytes) — used to read offset data from sensor NVM

### What we can rewrite
Everything else. The ST ULD is just I2C register reads/writes with:
- **Page select** via register 0x7FFF (bank switching)
- **DCI (Device Configuration Interface)** — a command/response protocol at addresses 0x2C00-0x2FFF
- **Polling loops** waiting for status bytes
- **Block headers** in output data (tagged TLV-style format)

### Key Protocol Observations

#### Page/Bank Select
Register 0x7FFF selects the I2C address space page. The sensor uses pages 0x00-0x0B.
Firmware upload writes to pages 0x09, 0x0A, 0x0B at address 0x0000.

#### DCI Protocol
Configuration read/write goes through a mailbox:
- **Write**: Build command at 0x2C04-0x2FFF (data + 4-byte header + 8-byte footer), then poll 0x2C00 for status == 0x03
- **Read**: Send read command at 0x2FF4-0x2FFF, poll for status, then read result from 0x2C04

Footer format for writes: `{0x00, 0x00, 0x00, 0x0F, 0x05, 0x01, size_hi, size_lo}`
Command format for reads: `{idx_hi, idx_lo, size_encoded_hi, size_encoded_lo, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x02, 0x00, 0x08}`

#### Data Ready Check
Read 4 bytes from address 0x0000:
- byte[0] = stream count (new != old and != 255)
- byte[1] == 0x05
- byte[2] & 0x05 == 0x05
- byte[3] & 0x10 == 0x10

#### Ranging Data Format
Read `data_read_size` bytes from 0x0000. Data is big-endian (needs swap on LE platforms).
Contains tagged blocks with Block_header at each section:
- idx identifies the data type (ambient, distance, sigma, etc.)
- type * size gives the block data length

Distance values are in units of 1/4 mm (divide by 4 for mm).
Sigma in units of 1/128 mm. Reflectance in units of 1/2 percent.

### DCI Register Map (key addresses)
| Address | Size | Description |
|---------|------|-------------|
| 0x5450 | 8 | Zone config (resolution) |
| 0x5458 | 4 | Ranging frequency (Hz) |
| 0x545C | 20 | Integration time (ms) |
| 0xAD30 | 8 | Ranging mode |
| 0xAD38 | 16 | DSS config |
| 0xAE64 | 4 | Target order |
| 0xAED8 | 16 | Sharpener |
| 0xB39C | 16 | Internal charge pump |
| 0xBFAC | varies | Motion detector config |
| 0xD964 | 4 | Single range mode |
| 0xD968 | 4 | Output config |
| 0xD970 | 8 | Output enables |
| 0xD980 | varies | Output list |
| 0xDB80 | 4 | Pipe control |

### Block Header IDX values for output data
| IDX | Data |
|-----|------|
| 0xD | Metadata (temp at offset +12) |
| See vl53l5cx_api.h for full list of _IDX defines |

## Lightweight Driver Design

### Class: `Adafruit_VL53L5_Lite`
```
class Adafruit_VL53L5_Lite {
public:
  bool begin(uint8_t addr = 0x29, TwoWire *wire = &Wire);

  // Core
  bool startRanging();
  bool stopRanging();
  bool isDataReady();
  bool getRangingData(int16_t *distances, uint8_t *statuses = nullptr);

  // Config
  bool setResolution(uint8_t res);  // 16 or 64
  uint8_t getResolution();
  bool setRangingFrequency(uint8_t hz);
  uint8_t getRangingFrequency();
  bool setIntegrationTime(uint32_t ms);
  uint32_t getIntegrationTime();
  bool setSharpenerPercent(uint8_t pct);
  uint8_t getSharpenerPercent();
  bool setTargetOrder(uint8_t order);
  uint8_t getTargetOrder();
  bool setRangingMode(uint8_t mode);
  uint8_t getRangingMode();
  bool setPowerMode(uint8_t mode);
  uint8_t getPowerMode();
  bool setAddress(uint8_t addr);

private:
  Adafruit_I2CDevice *_i2c;
  uint8_t _streamcount;
  uint32_t _data_read_size;
  uint8_t _resolution;

  // I2C helpers
  bool _writeByte(uint16_t reg, uint8_t val);
  bool _readByte(uint16_t reg, uint8_t *val);
  bool _writeMulti(uint16_t reg, const uint8_t *data, uint32_t len);
  bool _readMulti(uint16_t reg, uint8_t *data, uint32_t len);
  void _setPage(uint8_t page);

  // DCI helpers
  bool _dciRead(uint16_t index, uint8_t *data, uint16_t size);
  bool _dciWrite(const uint8_t *data, uint16_t index, uint16_t size);
  bool _dciReplace(uint16_t index, uint16_t total_size,
                   const uint8_t *data, uint16_t data_size, uint16_t offset);
  bool _pollForAnswer(uint8_t size, uint8_t pos, uint16_t reg,
                      uint8_t mask, uint8_t expected);
  bool _pollForBoot();

  // Init sequence
  bool _uploadFirmware();
  bool _sendDefaultConfig();
};
```

### RAM savings
- No pre-allocated temp_buffer (allocate on stack where needed, or use a smaller shared buffer)
- No pre-allocated offset_data/xtalk_data (read and write inline during init)
- Target: <1KB persistent RAM vs ~2.7KB for ST ULD

### Flash cost
- Firmware blob: 86KB (unavoidable)
- Default config: 971 bytes (unavoidable)
- Default xtalk: 776 bytes (unavoidable)
- Driver code: estimated ~3-4KB vs ~8KB for ST ULD
- Total savings: ~4KB flash, ~1.7KB RAM

### Approach
1. Start with init sequence (reboot, firmware upload, config, NVM/xtalk)
2. Add start/stop ranging
3. Add data ready check + data read (simplified — just distances + status)
4. Add config get/set via DCI
5. Test against hw_tests 01-08
