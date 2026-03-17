/*!
 * @file Adafruit_VL53L5_Lite.cpp
 *
 * Lightweight BusIO-native driver for VL53L5CX 8x8 ToF sensor.
 * Reimplements the ST ULD protocol using only Adafruit_I2CDevice.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 * MIT license
 */

#include "Adafruit_VL53L5_Lite.h"

// Block header indices for output data parsing
#define BH_IDX_METADATA 0xD
#define BH_IDX_AMBIENT_RATE 0x0
#define BH_IDX_NB_TARGET_DETECTED 0x1
#define BH_IDX_SPAD_COUNT 0x3
#define BH_IDX_SIGNAL_RATE 0x4
#define BH_IDX_RANGE_SIGMA 0x6
#define BH_IDX_DISTANCE 0x5
#define BH_IDX_REFLECTANCE 0x7
#define BH_IDX_TARGET_STATUS 0x8
#define BH_IDX_MOTION 0x9

// Glare filter DCI address
#define VL53L5_GLARE_FILTER 0xB870

// ============================================================
// Constructor / Destructor
// ============================================================

Adafruit_VL53L5_Lite::Adafruit_VL53L5_Lite() {}

Adafruit_VL53L5_Lite::~Adafruit_VL53L5_Lite() {
  if (_i2c) {
    delete _i2c;
  }
}

// ============================================================
// Low-level I2C helpers
// ============================================================

bool Adafruit_VL53L5_Lite::_writeByte(uint16_t reg, uint8_t val) {
  uint8_t buf[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val};
  return _i2c->write(buf, 3);
}

bool Adafruit_VL53L5_Lite::_readByte(uint16_t reg, uint8_t *val) {
  uint8_t r[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  return _i2c->write_then_read(r, 2, val, 1);
}

bool Adafruit_VL53L5_Lite::_writeMulti(uint16_t reg, const uint8_t *data,
                                        uint32_t len) {
  uint32_t maxPayload = _i2c->maxBufferSize() - 2; // reserve 2 for reg addr
  uint8_t buf[maxPayload + 2];
  uint32_t offset = 0;

  while (offset < len) {
    uint32_t chunk = min(maxPayload, len - offset);
    uint16_t addr = reg + offset;
    buf[0] = (uint8_t)(addr >> 8);
    buf[1] = (uint8_t)(addr & 0xFF);
    memcpy(buf + 2, data + offset, chunk);
    if (!_i2c->write(buf, chunk + 2)) {
      return false;
    }
    offset += chunk;
  }
  return true;
}

bool Adafruit_VL53L5_Lite::_readMulti(uint16_t reg, uint8_t *data,
                                       uint32_t len) {
  uint32_t maxRead = _i2c->maxBufferSize();
  uint32_t offset = 0;

  while (offset < len) {
    uint32_t chunk = min(maxRead, len - offset);
    uint16_t addr = reg + offset;
    uint8_t r[2] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF)};
    if (!_i2c->write_then_read(r, 2, data + offset, chunk)) {
      return false;
    }
    offset += chunk;
  }
  return true;
}

void Adafruit_VL53L5_Lite::_setPage(uint8_t page) {
  _writeByte(VL53L5_PAGE_REG, page);
}

// ============================================================
// Byte-swap for big-endian sensor data on little-endian host
// ============================================================

void Adafruit_VL53L5_Lite::_swapBuffer(uint8_t *buf, uint16_t size) {
  for (uint16_t i = 0; i < size; i += 4) {
    uint32_t tmp = ((uint32_t)buf[i] << 24) | ((uint32_t)buf[i + 1] << 16) |
                   ((uint32_t)buf[i + 2] << 8) | (uint32_t)buf[i + 3];
    memcpy(&buf[i], &tmp, 4);
  }
}

// ============================================================
// Polling helpers
// ============================================================

bool Adafruit_VL53L5_Lite::_pollForAnswer(uint8_t size, uint8_t pos,
                                           uint16_t reg, uint8_t mask,
                                           uint8_t expected) {
  uint16_t timeout = 0;
  uint8_t status;

  do {
    if (!_readMulti(reg, _temp, size)) {
      Serial.println(F("    [poll] readMulti failed"));
      return false;
    }
    if (size >= 4 && _temp[2] >= 0x7F) {
      Serial.print(F("    [poll] MCU error: 0x")); Serial.println(_temp[2], HEX);
      return false; // MCU error
    }
    if ((_temp[pos] & mask) == expected) {
      return true;
    }
    delay(1);
    timeout++;
  } while (timeout < 2000);

  Serial.print(F("    [poll] Timeout! Last val at pos ")); Serial.print(pos);
  Serial.print(F(": 0x")); Serial.println(_temp[pos], HEX);
  return false; // Timeout
}

bool Adafruit_VL53L5_Lite::_pollForBoot() {
  uint8_t go2_status0, go2_status1;
  uint16_t timeout = 0;

  do {
    if (!_readByte(0x06, &go2_status0)) {
      return false;
    }
    if (go2_status0 & 0x80) {
      _readByte(0x07, &go2_status1);
      return (go2_status1 == 0);
    }
    if (go2_status0 & 0x01) {
      return true;
    }
    delay(1);
    timeout++;
  } while (timeout < 500);

  return false;
}

// ============================================================
// DCI (Device Configuration Interface) protocol
// ============================================================

bool Adafruit_VL53L5_Lite::_dciRead(uint16_t index, uint8_t *data,
                                     uint16_t size) {
  uint32_t rd_size = (uint32_t)size + 12;
  if (rd_size > VL53L5_TEMP_BUFFER_SIZE) {
    return false;
  }

  uint8_t cmd[12] = {
      (uint8_t)(index >> 8),
      (uint8_t)(index & 0xFF),
      (uint8_t)((size & 0xFF0) >> 4),
      (uint8_t)((size & 0x0F) << 4),
      0x00, 0x00, 0x00, 0x0F, 0x00, 0x02, 0x00, 0x08};

  if (!_writeMulti(VL53L5_UI_CMD_END - 11, cmd, sizeof(cmd))) {
    return false;
  }
  if (!_pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03)) {
    return false;
  }
  if (!_readMulti(VL53L5_UI_CMD_START, _temp, rd_size)) {
    return false;
  }
  _swapBuffer(_temp, size + 12);
  memcpy(data, _temp + 4, size);
  return true;
}

bool Adafruit_VL53L5_Lite::_dciWrite(const uint8_t *data, uint16_t index,
                                      uint16_t size) {
  uint8_t headers[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t footer[8] = {0x00,
                        0x00,
                        0x00,
                        0x0F,
                        0x05,
                        0x01,
                        (uint8_t)((size + 8) >> 8),
                        (uint8_t)((size + 8) & 0xFF)};

  uint16_t address = VL53L5_UI_CMD_END - (size + 12) + 1;

  // Build command: header + index_encoded + data + footer
  headers[0] = (uint8_t)(index >> 8);
  headers[1] = (uint8_t)(index & 0xFF);
  headers[2] = (uint8_t)((size & 0xFF0) >> 4);
  headers[3] = (uint8_t)((size & 0x0F) << 4);

  // Write header
  _swapBuffer(headers, 4);
  if (!_writeMulti(address, headers, 4)) {
    return false;
  }

  // Write data (need to swap first — work on temp copy)
  if (size > VL53L5_TEMP_BUFFER_SIZE) {
    return false;
  }
  memcpy(_temp, data, size);
  _swapBuffer(_temp, size);
  if (!_writeMulti(address + 4, _temp, size)) {
    return false;
  }

  // Write footer
  if (!_writeMulti(address + 4 + size, footer, 8)) {
    return false;
  }

  return _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
}

bool Adafruit_VL53L5_Lite::_dciReplace(uint16_t index, uint16_t total_size,
                                        const uint8_t *data,
                                        uint16_t data_size, uint16_t offset) {
  // Read current value
  if (!_dciRead(index, _temp, total_size)) {
    return false;
  }
  // Replace bytes at offset
  memcpy(_temp + offset, data, data_size);
  // Write back
  return _dciWrite(_temp, index, total_size);
}

// ============================================================
// Init sequence
// ============================================================

bool Adafruit_VL53L5_Lite::_swReboot() {
  uint8_t tmp;

  _setPage(0x00);
  _writeByte(0x0009, 0x04);
  _writeByte(0x000F, 0x40);
  _writeByte(0x000A, 0x03);
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x000C, 0x01);

  _writeByte(0x0101, 0x00);
  _writeByte(0x0102, 0x00);
  _writeByte(0x010A, 0x01);
  _writeByte(0x4002, 0x01);
  _writeByte(0x4002, 0x00);
  _writeByte(0x010A, 0x03);
  _writeByte(0x0103, 0x01);
  _writeByte(0x000C, 0x00);
  _writeByte(0x000F, 0x43);
  delay(1);

  _writeByte(0x000F, 0x40);
  _writeByte(0x000A, 0x01);
  delay(100);

  // Wait for sensor boot
  _setPage(0x00);
  if (!_pollForAnswer(1, 0, 0x06, 0xFF, 1)) {
    return false;
  }

  _writeByte(0x000E, 0x01);
  _setPage(0x02);

  // Enable FW access
  _writeByte(0x03, 0x0D);
  _setPage(0x01);
  if (!_pollForAnswer(1, 0, 0x21, 0x10, 0x10)) {
    return false;
  }
  _setPage(0x00);

  // Enable host access to GO1
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x0C, 0x01);

  // Power ON status
  _setPage(0x00);
  _writeByte(0x101, 0x00);
  _writeByte(0x102, 0x00);
  _writeByte(0x010A, 0x01);
  _writeByte(0x4002, 0x01);
  _writeByte(0x4002, 0x00);
  _writeByte(0x010A, 0x03);
  _writeByte(0x103, 0x01);
  _writeByte(0x400F, 0x00);
  _writeByte(0x21A, 0x43);
  _writeByte(0x21A, 0x03);
  _writeByte(0x21A, 0x01);
  _writeByte(0x21A, 0x00);
  _writeByte(0x219, 0x00);
  _writeByte(0x21B, 0x00);

  // Wake up MCU
  _setPage(0x00);
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x0C, 0x00);
  _setPage(0x01);
  _writeByte(0x20, 0x07);
  _writeByte(0x20, 0x06);

  return true;
}

bool Adafruit_VL53L5_Lite::_uploadFirmware() {
  // Upload firmware in 3 chunks across pages 0x09, 0x0A, 0x0B
  _setPage(0x09);
  if (!_writeMulti(0, (uint8_t *)&VL53L5CX_FIRMWARE[0], 0x8000)) {
    return false;
  }
  _setPage(0x0A);
  if (!_writeMulti(0, (uint8_t *)&VL53L5CX_FIRMWARE[0x8000], 0x8000)) {
    return false;
  }
  _setPage(0x0B);
  if (!_writeMulti(0, (uint8_t *)&VL53L5CX_FIRMWARE[0x10000], 0x5000)) {
    return false;
  }
  _setPage(0x01);

  // Verify FW download
  _setPage(0x02);
  _writeByte(0x03, 0x0D);
  _setPage(0x01);
  if (!_pollForAnswer(1, 0, 0x21, 0x10, 0x10)) {
    return false;
  }

  uint8_t tmp;
  _setPage(0x00);
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x0C, 0x01);

  // Reset MCU and wait boot
  _setPage(0x00);
  _writeByte(0x114, 0x00);
  _writeByte(0x115, 0x00);
  _writeByte(0x116, 0x42);
  _writeByte(0x117, 0x00);
  _writeByte(0x0B, 0x00);
  _readByte(VL53L5_PAGE_REG, &tmp);
  _writeByte(0x0C, 0x00);
  _writeByte(0x0B, 0x01);

  if (!_pollForBoot()) {
    return false;
  }

  _setPage(0x02);
  return true;
}

bool Adafruit_VL53L5_Lite::_sendOffsetData(uint8_t resolution) {
  // Build offset command with signal and range grids from NVM data
  // This mirrors _vl53l5cx_send_offset_data in the ST ULD
  uint32_t signal_grid[64];
  int16_t range_grid[64];
  uint8_t dss_4x4[] = {0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07};
  uint8_t footer[] = {0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4};

  memcpy(signal_grid, &_offset_data[0x10], sizeof(signal_grid));
  memcpy(range_grid, &_offset_data[0x110], sizeof(range_grid));

  // Reorder from 8x8 to 4x4 lookup (mirroring ST logic)
  for (int i = 0; i < 16; i++) {
    // ST maps 4x4 zone indices to 8x8 positions
    signal_grid[i] = signal_grid[(4 * (i / 4)) * 8 + 4 * (i % 4)];
    range_grid[i] = range_grid[(4 * (i / 4)) * 8 + 4 * (i % 4)];
  }

  // Build the full command buffer
  memset(_temp, 0, VL53L5_TEMP_BUFFER_SIZE);
  memcpy(&_temp[0x00], _offset_data, 0x10);        // Header from NVM
  memcpy(&_temp[0x10], signal_grid, 16 * 4);        // 4x4 signal grid
  memcpy(&_temp[0x110], range_grid, 16 * 2);        // 4x4 range grid
  memcpy(&_temp[VL53L5_OFFSET_BUFFER_SIZE - 8], dss_4x4, 8);

  _swapBuffer(_temp, VL53L5_OFFSET_BUFFER_SIZE);

  uint16_t address = VL53L5_UI_CMD_END - (VL53L5_OFFSET_BUFFER_SIZE + 8) + 1;
  if (!_writeMulti(address, _temp, VL53L5_OFFSET_BUFFER_SIZE)) {
    return false;
  }
  if (!_writeMulti(address + VL53L5_OFFSET_BUFFER_SIZE, footer, 8)) {
    return false;
  }
  return _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
}

bool Adafruit_VL53L5_Lite::_sendXtalkData(uint8_t resolution) {
  uint8_t footer[] = {0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x03, 0x04};

  memcpy(_temp, _xtalk_data, VL53L5_XTALK_BUFFER_SIZE);
  _swapBuffer(_temp, VL53L5_XTALK_BUFFER_SIZE);

  uint16_t address = VL53L5_UI_CMD_END - (VL53L5_XTALK_BUFFER_SIZE + 8) + 1;
  Serial.print(F("    [xtalk] Writing ")); Serial.print(VL53L5_XTALK_BUFFER_SIZE);
  Serial.print(F(" bytes to 0x")); Serial.println(address, HEX);
  if (!_writeMulti(address, _temp, VL53L5_XTALK_BUFFER_SIZE)) {
    Serial.println(F("    [xtalk] Data write failed"));
    return false;
  }
  Serial.println(F("    [xtalk] Writing footer..."));
  if (!_writeMulti(address + VL53L5_XTALK_BUFFER_SIZE, footer, 8)) {
    Serial.println(F("    [xtalk] Footer write failed"));
    return false;
  }
  Serial.println(F("    [xtalk] Polling for status 0x03..."));
  bool ok = _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
  if (!ok) Serial.println(F("    [xtalk] Poll failed!"));
  return ok;
}

bool Adafruit_VL53L5_Lite::_loadNVMAndCalibration() {
  Serial.println(F("    [nvm] Writing NVM cmd..."));
  if (!_writeMulti(0x2FD8, (uint8_t *)VL53L5CX_GET_NVM_CMD,
                   sizeof(VL53L5CX_GET_NVM_CMD))) {
    Serial.println(F("    [nvm] Write NVM cmd failed"));
    return false;
  }
  Serial.println(F("    [nvm] Polling for NVM response..."));
  if (!_pollForAnswer(4, 0, VL53L5_UI_CMD_STATUS, 0xFF, 2)) {
    Serial.println(F("    [nvm] Poll for NVM failed"));
    return false;
  }
  Serial.println(F("    [nvm] Reading NVM data..."));
  if (!_readMulti(VL53L5_UI_CMD_START, _temp, VL53L5_NVM_DATA_SIZE)) {
    Serial.println(F("    [nvm] Read NVM data failed"));
    return false;
  }
  memcpy(_offset_data, _temp, VL53L5_OFFSET_BUFFER_SIZE);

  Serial.println(F("    [nvm] Sending offset data..."));
  if (!_sendOffsetData(VL53L5_RESOLUTION_4X4)) {
    Serial.println(F("    [nvm] Send offset failed"));
    return false;
  }

  Serial.println(F("    [nvm] Sending xtalk data..."));
  memcpy(_xtalk_data, (uint8_t *)VL53L5CX_DEFAULT_XTALK,
         VL53L5_XTALK_BUFFER_SIZE);
  if (!_sendXtalkData(VL53L5_RESOLUTION_4X4)) {
    Serial.println(F("    [nvm] Send xtalk failed"));
    return false;
  }
  Serial.println(F("    [nvm] Done!"));
  return true;
}

bool Adafruit_VL53L5_Lite::_sendDefaultConfig() {
  if (!_writeMulti(0x2C34, (uint8_t *)VL53L5CX_DEFAULT_CONFIGURATION,
                   sizeof(VL53L5CX_DEFAULT_CONFIGURATION))) {
    return false;
  }
  if (!_pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03)) {
    return false;
  }

  // Set pipe control (1 target per zone)
  uint8_t pipe_ctrl[] = {1, 0x00, 0x01, 0x00};
  if (!_dciWrite(pipe_ctrl, VL53L5_DCI_PIPE_CONTROL, sizeof(pipe_ctrl))) {
    return false;
  }

  // Set single range mode
  uint32_t single_range = 0x01;
  if (!_dciWrite((uint8_t *)&single_range, VL53L5_DCI_SINGLE_RANGE, 4)) {
    return false;
  }

  // Enable glare filter
  uint8_t one = 1;
  if (!_dciReplace(VL53L5_GLARE_FILTER, 40, &one, 1, 0x26)) {
    return false;
  }
  return _dciReplace(VL53L5_GLARE_FILTER, 40, &one, 1, 0x25);
}

// ============================================================
// Public API
// ============================================================

bool Adafruit_VL53L5_Lite::begin(uint8_t address, TwoWire *wire) {
  if (_i2c) {
    delete _i2c;
  }
  _i2c = new Adafruit_I2CDevice(address, wire);
  if (!_i2c->begin()) {
    Serial.println(F("  [lite] I2C begin failed"));
    return false;
  }

  // Check sensor is alive
  _setPage(0x00);
  uint8_t device_id = 0, revision_id = 0;
  _readByte(0x00, &device_id);
  _readByte(0x01, &revision_id);
  _setPage(0x02);

  Serial.print(F("  [lite] device_id=0x"));
  Serial.print(device_id, HEX);
  Serial.print(F(" revision_id=0x"));
  Serial.println(revision_id, HEX);

  if (device_id != 0xF0 || revision_id != 0x02) {
    Serial.println(F("  [lite] ID mismatch"));
    return false;
  }

  Serial.println(F("  [lite] Starting SW reboot..."));
  if (!_swReboot()) {
    Serial.println(F("  [lite] SW reboot failed"));
    return false;
  }

  Serial.println(F("  [lite] Uploading firmware..."));
  if (!_uploadFirmware()) {
    Serial.println(F("  [lite] Firmware upload failed"));
    return false;
  }

  Serial.println(F("  [lite] Loading NVM/calibration..."));
  if (!_loadNVMAndCalibration()) {
    Serial.println(F("  [lite] NVM/calibration failed"));
    return false;
  }

  Serial.println(F("  [lite] Sending default config..."));
  if (!_sendDefaultConfig()) {
    Serial.println(F("  [lite] Default config failed"));
    return false;
  }

  _initialized = true;
  _resolution = VL53L5_RESOLUTION_4X4;
  Serial.println(F("  [lite] Init complete!"));
  return true;
}

bool Adafruit_VL53L5_Lite::startRanging() {
  if (!_initialized) {
    return false;
  }

  uint8_t resolution;
  uint32_t i;

  resolution = getResolution();
  _data_read_size = 0;
  _streamcount = 255;

  // Read output config to determine data read size
  // This mirrors the ST start_ranging logic: compute header_config
  // and output list, then issue the start command
  uint32_t output[17] = {0};
  uint32_t header_config[2] = {0, 0};

  // Build output list based on enabled outputs
  output[0] = 0x0254; // Start BH for output list
  output[1] = 0x1040; // Metadata
  output[2] = 0x0840; // Common data
  output[3] = 0x40040; // Ambient rate
  output[4] = 0x10140; // Nb targets
  output[5] = 0x40540; // Spad count
  output[6] = 0x80440; // Signal rate
  output[7] = 0x20540; // Distance
  output[8] = 0x10640; // Range sigma
  output[9] = 0x10740; // Reflectance
  output[10] = 0x10840; // Target status
  // output[11-16] reserved
  output[11] = 0x10940; // Motion indicator

  // Set data size
  _data_read_size = 40; // headers + metadata + common

  // Per-zone data sizes
  uint32_t zones = (resolution == VL53L5_RESOLUTION_4X4) ? 16 : 64;
  _data_read_size += 260;  // ambient (4 bytes * 64 + 4 header)
  _data_read_size += 68;   // nb targets (1 * 64 + 4)
  _data_read_size += 260;  // spad count
  _data_read_size += 260;  // signal rate
  _data_read_size += 132;  // distance (2 * 64 + 4)
  _data_read_size += 132;  // sigma
  _data_read_size += 68;   // reflectance
  _data_read_size += 68;   // target status
  _data_read_size += 144;  // motion indicator
  _data_read_size += 20;   // footer

  header_config[0] = _data_read_size;
  header_config[1] = 16 + 12; // header + metadata + common

  // Write output settings via DCI
  if (!_dciWrite((uint8_t *)output, VL53L5_DCI_OUTPUT_LIST,
                 sizeof(output))) {
    return false;
  }
  if (!_dciWrite((uint8_t *)header_config, VL53L5_DCI_OUTPUT_CONFIG,
                 sizeof(header_config))) {
    return false;
  }

  // Enable all output block headers
  uint32_t output_bh_enable[4] = {0x00000007, 0x00000000, 0x00000000,
                                   0xC0000000};
  if (!_dciWrite((uint8_t *)output_bh_enable, VL53L5_DCI_OUTPUT_ENABLES,
                 sizeof(output_bh_enable))) {
    return false;
  }

  // Send start command
  uint8_t cmd[] = {0x00, 0x03, 0x00, 0x00};
  if (!_writeMulti(VL53L5_UI_CMD_END - 3, cmd, 4)) {
    return false;
  }
  return _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
}

bool Adafruit_VL53L5_Lite::stopRanging() {
  if (!_initialized) {
    return false;
  }

  uint8_t tmp = 0;
  uint16_t timeout = 0;

  // Issue stop command
  _readMulti(0x0000, _temp, 4);
  uint8_t auto_flag = _temp[3];

  if (auto_flag & 0x80) {
    // Autonomous mode — just send stop
    uint8_t cmd[] = {0x00, 0x02, 0x00, 0x00};
    _writeMulti(VL53L5_UI_CMD_END - 3, cmd, 4);
    _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
  } else {
    // Continuous mode — wait for data ready, then stop
    do {
      _readMulti(0x0000, _temp, 4);
      if ((_temp[0] != _streamcount) && (_temp[0] != 255) &&
          (_temp[1] == 0x05) && ((_temp[2] & 0x05) == 0x05) &&
          ((_temp[3] & 0x10) == 0x10)) {
        break;
      }
      delay(1);
      timeout++;
    } while (timeout < 1000);

    uint8_t cmd[] = {0x00, 0x02, 0x00, 0x00};
    _writeMulti(VL53L5_UI_CMD_END - 3, cmd, 4);
    _pollForAnswer(4, 1, VL53L5_UI_CMD_STATUS, 0xFF, 0x03);
  }

  return true;
}

bool Adafruit_VL53L5_Lite::isDataReady() {
  if (!_initialized) {
    return false;
  }

  uint8_t buf[4];
  if (!_readMulti(0x0000, buf, 4)) {
    return false;
  }

  if ((buf[0] != _streamcount) && (buf[0] != 255) && (buf[1] == 0x05) &&
      ((buf[2] & 0x05) == 0x05) && ((buf[3] & 0x10) == 0x10)) {
    _streamcount = buf[0];
    return true;
  }
  return false;
}

bool Adafruit_VL53L5_Lite::getRangingData(int16_t *distances,
                                            uint8_t *statuses,
                                            uint16_t *sigmas) {
  if (!_initialized || _data_read_size == 0 ||
      _data_read_size > VL53L5_TEMP_BUFFER_SIZE) {
    return false;
  }

  // Read raw data
  if (!_readMulti(0x0000, _temp, _data_read_size)) {
    return false;
  }
  _streamcount = _temp[0];
  _swapBuffer(_temp, (uint16_t)_data_read_size);

  // Parse tagged blocks
  uint8_t zones =
      (_resolution == VL53L5_RESOLUTION_8X8) ? 64 : 16;

  for (uint32_t i = 16; i < _data_read_size; i += 4) {
    // Block header: {idx(8), type(4), size(12), unused(8)}
    // After swap, it's in native u32 format
    uint32_t bh;
    memcpy(&bh, &_temp[i], 4);
    uint8_t idx = (bh >> 16) & 0xFF;
    uint32_t btype = (bh >> 12) & 0x0F;
    uint32_t bsize = bh & 0x0FFF;

    uint32_t msize;
    if (btype > 1 && btype < 0xD) {
      msize = btype * bsize;
    } else {
      msize = bsize;
    }

    switch (idx) {
    case BH_IDX_METADATA:
      _temperature = (int8_t)_temp[i + 12];
      break;
    case BH_IDX_DISTANCE:
      if (distances) {
        memcpy(distances, &_temp[i + 4], zones * 2);
        // Convert from 1/4 mm to mm
        for (uint8_t z = 0; z < zones; z++) {
          distances[z] /= 4;
          if (distances[z] < 0) {
            distances[z] = 0;
          }
        }
      }
      break;
    case BH_IDX_TARGET_STATUS:
      if (statuses) {
        memcpy(statuses, &_temp[i + 4], zones);
      }
      break;
    case BH_IDX_RANGE_SIGMA:
      if (sigmas) {
        memcpy(sigmas, &_temp[i + 4], zones * 2);
        // Convert from 1/128 mm to mm
        for (uint8_t z = 0; z < zones; z++) {
          sigmas[z] /= 128;
        }
      }
      break;
    default:
      break;
    }
    i += msize;
  }
  return true;
}

// ============================================================
// Configuration get/set via DCI
// ============================================================

bool Adafruit_VL53L5_Lite::setResolution(uint8_t res) {
  if (!_initialized ||
      (res != VL53L5_RESOLUTION_4X4 && res != VL53L5_RESOLUTION_8X8)) {
    return false;
  }

  if (res == VL53L5_RESOLUTION_4X4) {
    uint8_t dss[] = {0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (!_dciWrite(dss, VL53L5_DCI_DSS_CONFIG, 16)) {
      return false;
    }
    uint8_t zone[] = {0x04, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (!_dciWrite(zone, VL53L5_DCI_ZONE_CONFIG, 8)) {
      return false;
    }
  } else {
    uint8_t dss[] = {0x1F, 0x04, 0x04, 0x47, 0x08, 0x10, 0x10, 0x07,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (!_dciWrite(dss, VL53L5_DCI_DSS_CONFIG, 16)) {
      return false;
    }
    uint8_t zone[] = {0x08, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (!_dciWrite(zone, VL53L5_DCI_ZONE_CONFIG, 8)) {
      return false;
    }
  }

  _sendOffsetData(res);
  _sendXtalkData(res);
  _resolution = res;
  return true;
}

uint8_t Adafruit_VL53L5_Lite::getResolution() {
  if (!_initialized) {
    return 0;
  }
  uint8_t buf[8];
  if (!_dciRead(VL53L5_DCI_ZONE_CONFIG, buf, 8)) {
    return 0;
  }
  return buf[0] * buf[1]; // width * height
}

bool Adafruit_VL53L5_Lite::setRangingFrequency(uint8_t hz) {
  if (!_initialized) {
    return false;
  }
  uint32_t freq = hz;
  return _dciReplace(VL53L5_DCI_FREQ_HZ, 4, (uint8_t *)&freq, 4, 0);
}

uint8_t Adafruit_VL53L5_Lite::getRangingFrequency() {
  if (!_initialized) {
    return 0;
  }
  uint32_t freq = 0;
  if (!_dciRead(VL53L5_DCI_FREQ_HZ, (uint8_t *)&freq, 4)) {
    return 0;
  }
  return (uint8_t)freq;
}

bool Adafruit_VL53L5_Lite::setIntegrationTime(uint32_t ms) {
  if (!_initialized) {
    return false;
  }
  // Integration time is at offset 0 in a 20-byte DCI block
  // Value is in microseconds (* 1000)
  uint32_t us = ms * 1000;
  return _dciReplace(VL53L5_DCI_INT_TIME, 20, (uint8_t *)&us, 4, 0);
}

uint32_t Adafruit_VL53L5_Lite::getIntegrationTime() {
  if (!_initialized) {
    return 0;
  }
  uint8_t buf[20];
  if (!_dciRead(VL53L5_DCI_INT_TIME, buf, 20)) {
    return 0;
  }
  uint32_t us;
  memcpy(&us, buf, 4);
  return us / 1000; // Convert to ms
}

bool Adafruit_VL53L5_Lite::setSharpenerPercent(uint8_t pct) {
  if (!_initialized) {
    return false;
  }
  uint8_t zones = (_resolution == VL53L5_RESOLUTION_4X4) ? 16 : 64;
  uint8_t buf[16];
  if (!_dciRead(VL53L5_DCI_SHARPENER, buf, 16)) {
    return false;
  }
  // Sharpener target is stored as percentage * 256 / 100 at each zone offset
  for (uint8_t i = 0; i < zones; i++) {
    buf[i] = (pct * 256) / 100;
  }
  return _dciWrite(buf, VL53L5_DCI_SHARPENER, 16);
}

uint8_t Adafruit_VL53L5_Lite::getSharpenerPercent() {
  if (!_initialized) {
    return 0xFF;
  }
  uint8_t buf[16];
  if (!_dciRead(VL53L5_DCI_SHARPENER, buf, 16)) {
    return 0xFF;
  }
  return (buf[0] * 100) / 256;
}

bool Adafruit_VL53L5_Lite::setTargetOrder(uint8_t order) {
  if (!_initialized) {
    return false;
  }
  uint32_t val = order;
  return _dciReplace(VL53L5_DCI_TARGET_ORDER, 4, (uint8_t *)&val, 4, 0);
}

uint8_t Adafruit_VL53L5_Lite::getTargetOrder() {
  if (!_initialized) {
    return 0;
  }
  uint32_t val = 0;
  if (!_dciRead(VL53L5_DCI_TARGET_ORDER, (uint8_t *)&val, 4)) {
    return 0;
  }
  return (uint8_t)val;
}

bool Adafruit_VL53L5_Lite::setRangingMode(uint8_t mode) {
  if (!_initialized) {
    return false;
  }
  uint8_t buf[8];
  if (!_dciRead(VL53L5_DCI_RANGING_MODE, buf, 8)) {
    return false;
  }
  buf[0] = mode;
  return _dciWrite(buf, VL53L5_DCI_RANGING_MODE, 8);
}

uint8_t Adafruit_VL53L5_Lite::getRangingMode() {
  if (!_initialized) {
    return 0;
  }
  uint8_t buf[8];
  if (!_dciRead(VL53L5_DCI_RANGING_MODE, buf, 8)) {
    return 0;
  }
  return buf[0];
}

bool Adafruit_VL53L5_Lite::setPowerMode(uint8_t mode) {
  if (!_initialized) {
    return false;
  }
  uint8_t current = getPowerMode();
  if (current == mode) {
    return true;
  }

  if (mode == VL53L5_POWER_MODE_WAKEUP) {
    _setPage(0x00);
    _writeByte(0x09, 0x04);
    if (!_pollForAnswer(1, 0, 0x06, 0x01, 1)) {
      return false;
    }
  } else if (mode == VL53L5_POWER_MODE_SLEEP) {
    _setPage(0x00);
    _writeByte(0x09, 0x02);
    if (!_pollForAnswer(1, 0, 0x06, 0x01, 0)) {
      return false;
    }
  } else {
    return false;
  }

  _setPage(0x02);
  return true;
}

uint8_t Adafruit_VL53L5_Lite::getPowerMode() {
  if (!_initialized) {
    return 0xFF;
  }

  uint8_t tmp;
  _setPage(0x00);
  _readByte(0x009, &tmp);
  _setPage(0x02);

  if (tmp == 0x04) {
    return VL53L5_POWER_MODE_WAKEUP;
  } else if (tmp == 0x02) {
    return VL53L5_POWER_MODE_SLEEP;
  }
  return 0xFF;
}

bool Adafruit_VL53L5_Lite::setAddress(uint8_t new_address) {
  if (!_initialized) {
    return false;
  }

  _setPage(0x00);
  _writeByte(0x04, new_address);
  _setPage(0x02);

  delete _i2c;
  _i2c = new Adafruit_I2CDevice(new_address, &Wire);
  return _i2c->begin();
}
