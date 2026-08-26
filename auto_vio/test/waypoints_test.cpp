#include <cassert>
#include <cstdio>
#include <cmath>
#include "../nav_math.h"

int main() {
  // Clean round-number case: field 10ft, bar 2ft, no overlap margin.
  Waypoint wp[32];
  int count = buildWaypoints(10.0f, 2.0f, 0.0f, wp, 32);

  Waypoint expected[] = {
    {0, 10}, {2, 10}, {2, 0}, {4, 0}, {4, 10},
    {6, 10}, {6, 0}, {8, 0}, {8, 10}, {10, 10}, {10, 0}
  };
  int expectedCount = sizeof(expected) / sizeof(Waypoint);
  assert(count == expectedCount);
  for (int i = 0; i < expectedCount; i++) {
    assert(fabsf(wp[i].x - expected[i].x) < 0.001f);
    assert(fabsf(wp[i].y - expected[i].y) < 0.001f);
  }

  // Real-world case: 480 sqft field, 17in bar, 15% overlap margin.
  // Must never leave a gap wider than the bar, and must fully span the field.
  const float FIELD = 21.91f;
  const float BAR = 17.0f / 12.0f;
  Waypoint real[64];
  int realCount = buildWaypoints(FIELD, BAR, 0.15f, real, 64);
  assert(realCount > 0);
  assert(fabsf(real[0].x - 0.0f) < 0.01f);
  assert(fabsf(real[realCount - 1].x - FIELD) < 0.01f);

  float prevX = -1.0f;
  for (int i = 0; i < realCount; i++) {
    if (real[i].x != prevX) {
      if (prevX >= 0.0f) {
        float gap = real[i].x - prevX;
        assert(gap <= BAR + 0.001f);
      }
      prevX = real[i].x;
    }
  }

  printf("waypoints_test: all assertions passed (round-number count=%d, real-world count=%d)\n",
         count, realCount);
  return 0;
}
