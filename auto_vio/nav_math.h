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

struct Waypoint {
  float x;
  float y;
};

inline int buildWaypoints(float fieldSideFt, float barWidthFt, float overlapFraction,
                           Waypoint* out, int maxOut) {
  float spacingTarget = barWidthFt * (1.0f - overlapFraction);
  int lanes = (int)ceilf(fieldSideFt / spacingTarget) + 1;
  if (lanes < 2) lanes = 2;
  float spacing = fieldSideFt / (float)(lanes - 1);

  auto endY = [&](int lane) -> float {
    return (lane % 2 == 0) ? fieldSideFt : 0.0f;
  };

  int idx = 0;
  if (idx < maxOut) out[idx++] = { 0.0f, endY(0) };
  for (int lane = 1; lane < lanes && idx + 1 < maxOut; lane++) {
    float x2 = lane * spacing;
    out[idx++] = { x2, endY(lane - 1) };
    out[idx++] = { x2, endY(lane) };
  }
  return idx;
}
