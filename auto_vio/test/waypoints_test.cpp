#include <cassert>
#include <cstdio>
#include <cmath>
#include "../nav_math.h"

static void laneGeometryTests() {
  const float BAR = 2.0f;

  // 10ft wide, 2ft bar, no overlap -> 5 lanes tiling 0..10 exactly.
  assert(laneCount(10.0f, BAR, 0.0f) == 5);
  assert(fabsf(laneCenterX(0, BAR, 0.0f) - 1.0f) < 0.001f);
  assert(fabsf(laneCenterX(4, BAR, 0.0f) - 9.0f) < 0.001f);

  // Sprayed band must stay inside the rectangle on both edges.
  int n = laneCount(10.0f, BAR, 0.0f);
  assert(laneCenterX(0, BAR, 0.0f) - BAR / 2 >= -0.001f);
  assert(laneCenterX(n - 1, BAR, 0.0f) + BAR / 2 <= 10.0f + 0.001f);

  // Width not divisible by the bar: cover as much as fits, never overrun.
  int m = laneCount(9.0f, BAR, 0.0f);
  assert(m == 4);
  assert(laneCenterX(m - 1, BAR, 0.0f) + BAR / 2 <= 9.0f + 0.001f);

  // Narrower than the bar still yields one pass rather than zero.
  assert(laneCount(1.0f, BAR, 0.0f) == 1);

  // Real rover: 17in bar, 15% overlap, 21.91ft wide.
  const float REALBAR = 17.0f / 12.0f;
  int r = laneCount(21.91f, REALBAR, 0.15f);
  assert(r >= 2);
  assert(laneCenterX(r - 1, REALBAR, 0.15f) + REALBAR / 2 <= 21.91f + 0.001f);
  // and the next lane would have overrun, i.e. we did not stop early
  assert(laneCenterX(r, REALBAR, 0.15f) + REALBAR / 2 > 21.91f);

  printf("lane geometry: passed (10ft=%d lanes, 9ft=%d, real=%d)\n", n, m, r);
}

int main() {
  laneGeometryTests();

  // Clean round-number case: 10ft x 10ft field, bar 2ft, no overlap margin.
  Waypoint wp[32];
  int count = buildWaypoints(10.0f, 10.0f, 2.0f, 0.0f, wp, 32);

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
  int realCount = buildWaypoints(FIELD, FIELD, BAR, 0.15f, real, 64);
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

  // Rectangular field: lanes span the width, passes run the length.
  Waypoint rect[64];
  int rectCount = buildWaypoints(6.0f, 20.0f, 2.0f, 0.0f, rect, 64);
  assert(rectCount > 0);
  assert(fabsf(rect[0].x - 0.0f) < 0.01f);
  assert(fabsf(rect[0].y - 20.0f) < 0.01f);          // first pass runs the length
  assert(fabsf(rect[rectCount - 1].x - 6.0f) < 0.01f); // last lane reaches the width

  printf("waypoints_test: all assertions passed (round=%d, real=%d, rect=%d)\n",
         count, realCount, rectCount);
  return 0;
}
