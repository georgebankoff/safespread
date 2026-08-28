#include <cassert>
#include <cstdio>
#include <cmath>
#include "../nav_math.h"

int main() {
  assert(fabsf(angleDiffDeg(10.0f, 350.0f) - 20.0f) < 0.01f);
  assert(fabsf(angleDiffDeg(350.0f, 10.0f) - (-20.0f)) < 0.01f);
  assert(fabsf(angleDiffDeg(100.0f, 90.0f) - 10.0f) < 0.01f);

  assert(fabsf(bearingToWaypointDeg(0.0f, 1.0f) - 0.0f) < 0.01f);
  assert(fabsf(bearingToWaypointDeg(1.0f, 0.0f) - 90.0f) < 0.01f);
  assert(fabsf(bearingToWaypointDeg(0.0f, -1.0f) - 180.0f) < 0.01f);
  assert(fabsf(bearingToWaypointDeg(-1.0f, 0.0f) - 270.0f) < 0.01f);

  assert(waypointReached(0.3f, 0.3f, 0.5f));
  assert(!waypointReached(0.4f, 0.4f, 0.5f));

  printf("nav_test: all assertions passed\n");
  return 0;
}
