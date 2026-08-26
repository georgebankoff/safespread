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

// 11-byte '!D' packet: field dimensions in feet, set from the app.
inline bool parseAreaPacket(const uint8_t* d, size_t n, float& widthFt, float& lengthFt) {
  if (n < 11 || d[0] != 0x21 || d[1] != 0x44) return false;

  uint8_t sum = 0;
  for (int i = 0; i < 10; i++) sum += d[i];
  if ((uint8_t)(~sum) != d[10]) return false;

  memcpy(&widthFt, d + 2, 4);
  memcpy(&lengthFt, d + 6, 4);
  return true;
}

// --- Lane geometry -------------------------------------------------------
// Lanes are spaced so the spray bar tiles the width with a little overlap.
// The first lane sits at x=0, i.e. exactly where the rover is standing when
// the mission starts: placing the rover is how you aim the first pass, and it
// must therefore drive straight ahead rather than immediately steering onto a
// lane offset from it. Coverage consequently begins half a bar-width to the
// left of the start point. Lanes stop before the far edge, so a width not
// divisible by the bar is under-covered at the far side, never overrun.

inline float laneSpacing(float barWidthFt, float overlapFraction) {
  float s = barWidthFt * (1.0f - overlapFraction);
  return (s > 0.01f) ? s : 0.01f;
}

inline float laneCenterX(int lane, float barWidthFt, float overlapFraction) {
  return lane * laneSpacing(barWidthFt, overlapFraction);
}

inline int laneCount(float widthFt, float barWidthFt, float overlapFraction) {
  if (widthFt <= barWidthFt) return 1;
  float usable = widthFt - barWidthFt;
  int n = (int)floorf(usable / laneSpacing(barWidthFt, overlapFraction)) + 1;
  return (n < 1) ? 1 : n;
}

struct Waypoint {
  float x;
  float y;
};

inline int buildWaypoints(float widthFt, float lengthFt, float barWidthFt,
                           float overlapFraction, Waypoint* out, int maxOut) {
  float spacingTarget = barWidthFt * (1.0f - overlapFraction);
  int lanes = (int)ceilf(widthFt / spacingTarget) + 1;
  if (lanes < 2) lanes = 2;
  float spacing = widthFt / (float)(lanes - 1);

  auto endY = [&](int lane) -> float {
    return (lane % 2 == 0) ? lengthFt : 0.0f;
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

inline float angleDiffDeg(float target, float current) {
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

inline float bearingToWaypointDeg(float dx, float dy) {
  float b = atan2f(dx, dy) * (180.0f / (float)M_PI);
  if (b < 0.0f) b += 360.0f;
  return b;
}

inline bool waypointReached(float dx, float dy, float toleranceFt) {
  return (dx * dx + dy * dy) <= (toleranceFt * toleranceFt);
}
