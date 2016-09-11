#ifdef THIS_IS_NOT_THE_FIRMWARE
#include <rover12_comm/rover12_comm.h>
#else
#include "rover12_comm.h"
#include <avr/pgmspace.h>
#endif

namespace rover12_comm {

namespace {

uint32_t crc32step(uint32_t crc, uint8_t data) {
#ifdef THIS_IS_NOT_THE_FIRMWARE
  static const uint32_t table[16] = {
#else
  static PROGMEM const uint32_t table[16] = {
#endif
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  uint8_t table_idx = crc ^ (data >> (0 * 4));
#ifdef THIS_IS_NOT_THE_FIRMWARE
  crc = table[table_idx & 0x0f] ^ (crc >> 4);
#else
  crc = pgm_read_dword_near(table + (table_idx & 0x0f)) ^ (crc >> 4);
#endif
  table_idx = crc ^ (data >> (1 * 4));
#ifdef THIS_IS_NOT_THE_FIRMWARE
  crc = table[table_idx & 0x0f] ^ (crc >> 4);
#else
  crc = pgm_read_dword_near(table + (table_idx & 0x0f)) ^ (crc >> 4);
#endif
  return crc;
}

} // namespace

uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc = crc32step(crc, data[i]);
  }
  return ~crc;
}

void cobs(uint8_t* data, size_t len) {
  size_t current_idx = 1;
  const size_t end_idx = len + 1;
  size_t code_idx = 0;
  uint8_t code = 1;

  while (current_idx < end_idx) {
    if (data[current_idx] == 0) {
      data[code_idx] = code;
      code_idx = current_idx;
      code = 1;
    } else {
      ++code;
    }
    ++current_idx;
  }
  data[code_idx] = code;
}

void uncobs(uint8_t* data, size_t len) {
  size_t current_idx = 0;
  const size_t end_idx = len + 1;
  while (current_idx < end_idx) {
    size_t code_idx = current_idx;
    uint8_t code = data[code_idx];
    current_idx += code;
    data[code_idx] = 0;
  }
}

} // namespace rover12_comm
