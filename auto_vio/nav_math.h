#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>

inline bool parsePosePacket(const uint8_t* d, size_t n, float& x, float& y, float& heading) {
  if (n < 15 || d[0] != 0x21 || d[1] != 0x50) return false;

  uint8_t sum = 0;
  for (int i = 0; i < 14; i++) sum += d[i];
  uint8_t crc = (uint8_t)(~sum);
  if (crc != d[14]) return false;

  memcpy(&x, d + 2, 4);
  memcpy(&y, d + 6, 4);
  memcpy(&heading, d + 10, 4);
  return true;
}
