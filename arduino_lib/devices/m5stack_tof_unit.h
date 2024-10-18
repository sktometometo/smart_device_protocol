/*
 * From https://github.com/m5stack/M5Stack/blob/master/examples/Unit/ToF_VL53L0X/ToF_VL53L0X.ino
 */

#include <Arduino.h>
#if defined(M5STACK_FIRE)
#include <M5Stack.h>
#elif defined(M5STACK_CORE2)
#include <M5Core2.h>
#endif

#include <optional>

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID 0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14
#define VL53L0X_I2C_address 0x29  // I2C VL53L0X_I2C_address

byte gbuf[16];

uint16_t _bswap(byte b[]) {
  // Big Endian unsigned short to little endian unsigned short
  uint16_t val = ((b[0] << 8) & b[1]);
  return val;
}

uint16_t _makeuint16(int lsb, int msb) {
  return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void _write_byte_data(byte data) {
  Wire.beginTransmission(VL53L0X_I2C_address);
  Wire.write(data);
  Wire.endTransmission();
}

void _write_byte_data_at(byte reg, byte data) {
  // write data word at VL53L0X_I2C_address and register
  Wire.beginTransmission(VL53L0X_I2C_address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void _write_word_data_at(byte reg, uint16_t data) {
  // write data word at VL53L0X_I2C_address and register
  byte b0 = (data & 0xFF);
  byte b1 = ((data >> 8) && 0xFF);

  Wire.beginTransmission(VL53L0X_I2C_address);
  Wire.write(reg);
  Wire.write(b0);
  Wire.write(b1);
  Wire.endTransmission();
}

byte _read_byte_data() {
  Wire.requestFrom(VL53L0X_I2C_address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

byte _read_byte_data_at(byte reg) {
  // _write_byte_data((byte)0x00);
  _write_byte_data(reg);
  Wire.requestFrom(VL53L0X_I2C_address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

uint16_t _read_word_data_at(byte reg) {
  _write_byte_data(reg);
  Wire.requestFrom(VL53L0X_I2C_address, 2);
  while (Wire.available() < 2) delay(1);
  gbuf[0] = Wire.read();
  gbuf[1] = Wire.read();
  return _bswap(gbuf);
}

void _read_block_data_at(byte reg, int sz) {
  int i = 0;
  _write_byte_data(reg);
  Wire.requestFrom(VL53L0X_I2C_address, sz);
  for (i = 0; i < sz; i++) {
    while (Wire.available() < 1) delay(1);
    gbuf[i] = Wire.read();
  }
}

uint16_t _VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
  // Converts the encoded VCSEL period register value into the real
  // period in PLL clocks
  uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
  return vcsel_period_pclks;
}

bool init_m5stack_tof_unit() {
  Wire.begin();
  return true;
}

std::optional<std::tuple<uint16_t, uint16_t, uint16_t, byte>> get_m5stack_tof_unit_data() {
  _write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

  byte val = 0;
  int cnt = 0;
  while (cnt < 100) {  // 1 second waiting time max
    delay(10);
    val = _read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
    if (val & 0x01) break;
    cnt++;
  }
  if (val & 0x01) {
    // Serial.println("ready");
  } else {
    // Serial.println("not ready");
    return std::nullopt;
  }
  _read_block_data_at(0x14, 12);
  uint16_t acnt = _makeuint16(gbuf[7], gbuf[6]);
  uint16_t scnt = _makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = _makeuint16(gbuf[11], gbuf[10]);
  byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
  return std::make_tuple(acnt, scnt, dist, DeviceRangeStatusInternal);
}

// void loop() {
//   _read_block_data_at(0x14, 12);
//   uint16_t acnt = _makeuint16(gbuf[7], gbuf[6]);
//   uint16_t scnt = _makeuint16(gbuf[9], gbuf[8]);
//   uint16_t dist = _makeuint16(gbuf[11], gbuf[10]);
//   byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
//   M5.Lcd.fillRect(0, 35, 319, 239, BLACK);
//   M5.Lcd.setCursor(0, 35, 4);
//   M5.Lcd.print("ambient count: ");
//   M5.Lcd.println(acnt);
//   M5.Lcd.print("signal count: ");
//   M5.Lcd.println(scnt);
//   M5.Lcd.print("distance: ");
//   M5.Lcd.println(dist);
//   M5.Lcd.print("status: ");
//   M5.Lcd.println(DeviceRangeStatusInternal);
//   delay(1000);
// }