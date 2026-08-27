#pragma once

#include <cmath>
#include <limits>

struct SteeringKnot {
  int pulseUs;
  float curvaturePerFt;
};

inline bool validSteeringMap(const SteeringKnot *knots, int count) {
  if (knots == nullptr || count < 3) return false;
  bool pulseIncreasing = false;
  bool pulseDecreasing = false;
  bool hasStraight = false;

  for (int index = 0; index < count; ++index) {
    if (!std::isfinite(knots[index].curvaturePerFt)) return false;
    if (knots[index].curvaturePerFt == 0.0f) hasStraight = true;
    if (index == 0) continue;
    if (!(knots[index].curvaturePerFt > knots[index - 1].curvaturePerFt)) {
      return false;
    }
    if (knots[index].pulseUs > knots[index - 1].pulseUs) pulseIncreasing = true;
    if (knots[index].pulseUs < knots[index - 1].pulseUs) pulseDecreasing = true;
    if (knots[index].pulseUs == knots[index - 1].pulseUs) return false;
  }
  return hasStraight && knots[0].curvaturePerFt < 0.0f &&
         knots[count - 1].curvaturePerFt > 0.0f &&
         pulseIncreasing != pulseDecreasing;
}

inline float pulseForCurvature(const SteeringKnot *knots, int count,
                               float requested) {
  if (!validSteeringMap(knots, count) || !std::isfinite(requested)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  if (requested <= knots[0].curvaturePerFt) {
    return static_cast<float>(knots[0].pulseUs);
  }
  if (requested >= knots[count - 1].curvaturePerFt) {
    return static_cast<float>(knots[count - 1].pulseUs);
  }

  for (int index = 1; index < count; ++index) {
    if (requested > knots[index].curvaturePerFt) continue;
    const SteeringKnot &a = knots[index - 1];
    const SteeringKnot &b = knots[index];
    const float fraction = (requested - a.curvaturePerFt) /
                           (b.curvaturePerFt - a.curvaturePerFt);
    return a.pulseUs + fraction * static_cast<float>(b.pulseUs - a.pulseUs);
  }
  return static_cast<float>(knots[count - 1].pulseUs);
}
