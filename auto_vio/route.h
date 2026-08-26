#pragma once
#include "nav_math.h"
#include "dubins.h"

// The whole mission, computed once before the rover moves: a dense list of
// points to follow, each flagged for spray. Planning up front is what stops
// the rover re-deciding its route from its own tracking error, which is how
// small position errors used to turn into completely different maneuvers.

struct RoutePoint {
  float x;
  float y;
  bool spray;
};

const float ROUTE_STEP_FT = 0.5f;

// Rover heading (0 = +Y, clockwise) <-> maths heading (0 = +X, CCW).
inline float headingToMath(float headingDeg) {
  return (90.0f - headingDeg) * (float)M_PI / 180.0f;
}

/** Lanes are visited a turning-diameter apart so each U-turn is a simple half
 *  circle rather than a loop-out-and-back, then the skipped ones are picked up
 *  on later sweeps. Returns how many lane indices were written. */
inline int buildLaneOrder(int totalLanes, int skip, int *out, int maxOut) {
  if (skip < 1) skip = 1;
  int n = 0;
  for (int offset = 0; offset < skip && n < maxOut; offset++) {
    for (int lane = offset; lane < totalLanes && n < maxOut; lane += skip) {
      out[n++] = lane;
    }
  }
  return n;
}

inline int laneSkipFor(float turnRadiusFt, float barWidthFt, float overlapFraction,
                       int totalLanes) {
  float s = laneSpacing(barWidthFt, overlapFraction);
  int k = (int)lroundf((2.0f * turnRadiusFt) / s);
  if (k < 1) k = 1;
  if (totalLanes > 1 && k > totalLanes - 1) k = totalLanes - 1;
  return k;
}

/** Emit the full route: each lane driven end to end with spray on, joined by
 *  Dubins transits with spray off. Returns the number of points written. */
inline int buildRoute(float fieldPassFt, float fieldWidthFt,
                      float barWidthFt, float overlapFraction,
                      float turnRadiusFt,
                      RoutePoint *out, int maxOut) {
  int totalLanes = laneCount(fieldWidthFt, barWidthFt, overlapFraction);
  if (totalLanes < 1 || maxOut < 2) return 0;

  int order[64];
  int lanesToDrive = buildLaneOrder(totalLanes, laneSkipFor(turnRadiusFt, barWidthFt,
                                                            overlapFraction, totalLanes),
                                    order, 64);

  int n = 0;
  bool haveExit = false;
  float exitX = 0, exitY = 0, exitHeading = 0;

  for (int i = 0; i < lanesToDrive && n < maxOut; i++) {
    int lane = order[i];
    float laneX = laneCenterX(lane, barWidthFt, overlapFraction);

    // Alternate direction so consecutive passes run opposite ways, which is
    // what makes a U-turn the right join between them.
    bool goesUp = (i % 2) == 0;
    float startY = goesUp ? 0.0f : fieldPassFt;
    float endY = goesUp ? fieldPassFt : 0.0f;
    float heading = goesUp ? 0.0f : 180.0f;

    // Transit from the end of the previous pass to the start of this one.
    if (haveExit) {
      DubinsPath path;
      bool ok = dubinsCompute(exitX, exitY, headingToMath(exitHeading),
                              laneX, startY, headingToMath(heading),
                              turnRadiusFt, path);
      if (ok) {
        float len = dubinsLength(path);
        for (float s = ROUTE_STEP_FT; s < len && n < maxOut; s += ROUTE_STEP_FT) {
          float px, py, pth;
          dubinsPoseAt(path, s, px, py, pth);
          out[n].x = px;
          out[n].y = py;
          out[n].spray = false;  // never spray through a turn
          n++;
        }
      }
    }

    // The pass itself.
    float dir = goesUp ? ROUTE_STEP_FT : -ROUTE_STEP_FT;
    int steps = (int)(fieldPassFt / ROUTE_STEP_FT) + 1;
    for (int s = 0; s <= steps && n < maxOut; s++) {
      float y = startY + dir * s;
      if (goesUp ? (y > endY) : (y < endY)) y = endY;
      out[n].x = laneX;
      out[n].y = y;
      out[n].spray = true;
      n++;
      if (fabsf(y - endY) < 1e-4f) break;
    }

    exitX = laneX;
    exitY = endY;
    exitHeading = heading;
    haveExit = true;
  }

  return n;
}
