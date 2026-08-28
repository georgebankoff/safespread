#include <cassert>
#include <cmath>
#include <cstdio>
#include "../steering_map.h"

int main() {
  const SteeringKnot measured[] = {
      {2390, -1.0f / 4.33f},
      {1709, 0.0f},
      {700, 1.0f / 2.92f},
  };
  assert(validSteeringMap(measured, 3));

  // Straight is an explicit measured knot, not an inferred midpoint.
  assert(std::fabs(pulseForCurvature(measured, 3, 0.0f) - 1709.0f) < 0.001f);

  const float leftMid = 0.5f * measured[0].curvaturePerFt;
  const float rightMid = 0.5f * measured[2].curvaturePerFt;
  assert(std::fabs(pulseForCurvature(measured, 3, leftMid) - 2049.5f) < 0.01f);
  assert(std::fabs(pulseForCurvature(measured, 3, rightMid) - 1204.5f) < 0.01f);

  // Requests beyond the measured envelope clamp to a measured endpoint.
  assert(pulseForCurvature(measured, 3, -10.0f) == 2390.0f);
  assert(pulseForCurvature(measured, 3, 10.0f) == 700.0f);

  // Equal curvature magnitude produces deliberately asymmetric pulses because
  // the measured left and right steering curves are not mirror images.
  const float common = 0.30f;
  const float leftPulse = pulseForCurvature(measured, 3, -common);
  const float rightPulse = pulseForCurvature(measured, 3, common);
  assert(std::fabs((leftPulse - 1709.0f) - (1709.0f - rightPulse)) > 100.0f);

  // Curvature must rise strictly, pulse must be monotonic in one direction,
  // and a direct zero-curvature knot is mandatory.
  const SteeringKnot badCurvature[] = {
      {2200, -0.2f}, {1700, 0.1f}, {800, 0.05f},
  };
  const SteeringKnot badPulse[] = {
      {2200, -0.2f}, {1600, 0.0f}, {1800, 0.2f},
  };
  const SteeringKnot missingStraight[] = {
      {2200, -0.2f}, {1800, -0.1f}, {800, 0.2f},
  };
  const SteeringKnot onlyRight[] = {
      {1700, 0.0f}, {1200, 0.1f}, {800, 0.2f},
  };
  const SteeringKnot onlyLeft[] = {
      {2200, -0.2f}, {1900, -0.1f}, {1700, 0.0f},
  };
  assert(!validSteeringMap(badCurvature, 3));
  assert(!validSteeringMap(badPulse, 3));
  assert(!validSteeringMap(missingStraight, 3));
  assert(!validSteeringMap(onlyRight, 3));
  assert(!validSteeringMap(onlyLeft, 3));
  assert(std::isnan(pulseForCurvature(badPulse, 3, 0.0f)));

  const SteeringKnot increasingPulse[] = {
      {700, -0.3f}, {1500, 0.0f}, {2300, 0.3f},
  };
  assert(validSteeringMap(increasingPulse, 3));

  std::printf("steering_map_test: all assertions passed\n");
  return 0;
}
