#include <cassert>
#include <cstdio>
#include <cmath>
#include "../route.h"

int main() {
  const float BAR = 17.0f / 12.0f;
  const float OVERLAP = 0.15f;
  const float R = 4.0f;

  // --- lane ordering -----------------------------------------------------
  {
    int order[64];
    int n = buildLaneOrder(8, 3, order, 64);
    assert(n == 8);
    // Every lane exactly once.
    int seen[8] = {0};
    for (int i = 0; i < n; i++) { assert(order[i] >= 0 && order[i] < 8); seen[order[i]]++; }
    for (int i = 0; i < 8; i++) assert(seen[i] == 1);
    // Starts at the rover's own lane.
    assert(order[0] == 0);
  }

  // A skip of 1 must still be a valid ordering, not an empty one.
  {
    int order[64];
    assert(buildLaneOrder(5, 1, order, 64) == 5);
  }

  // --- skip follows the turning circle -----------------------------------
  {
    // 8ft circle against ~1.2ft lanes should skip several, not one.
    int k = laneSkipFor(R, BAR, OVERLAP, 18);
    assert(k >= 5 && k <= 8);
    // A rover that turned on a dime would take lanes in order.
    assert(laneSkipFor(0.3f, BAR, OVERLAP, 18) == 1);
  }

  // --- full route on the real field --------------------------------------
  {
    static RoutePoint pts[4000];
    int n = buildRoute(21.91f, 21.91f, BAR, OVERLAP, R, pts, 4000);
    assert(n > 100);

    int totalLanes = laneCount(21.91f, BAR, OVERLAP);

    // Every lane must actually get sprayed somewhere along its length.
    for (int lane = 0; lane < totalLanes; lane++) {
      float laneX = laneCenterX(lane, BAR, OVERLAP);
      bool covered = false;
      for (int i = 0; i < n; i++) {
        if (pts[i].spray && fabsf(pts[i].x - laneX) < 0.05f) { covered = true; break; }
      }
      assert(covered);
    }

    // Spray must never be commanded outside the rectangle.
    for (int i = 0; i < n; i++) {
      if (!pts[i].spray) continue;
      assert(pts[i].y >= -0.01f && pts[i].y <= 21.91f + 0.01f);
      assert(pts[i].x >= -0.01f && pts[i].x <= 21.91f + 0.01f);
    }

    // Consecutive points stay close enough to follow, so the tracker always
    // has a nearby target rather than a jump across the field.
    for (int i = 1; i < n; i++) {
      float dx = pts[i].x - pts[i - 1].x;
      float dy = pts[i].y - pts[i - 1].y;
      assert(sqrtf(dx * dx + dy * dy) <= ROUTE_STEP_FT * 2.5f);
    }

    // The route begins where the rover is placed: origin, first lane.
    assert(fabsf(pts[0].x) < 0.01f);
    assert(fabsf(pts[0].y) < 0.01f);
    assert(pts[0].spray);

    printf("route_test: real field -> %d points, %d lanes, skip %d\n",
           n, totalLanes, laneSkipFor(R, BAR, OVERLAP, totalLanes));
  }

  // --- a small plot still produces a complete route ----------------------
  {
    static RoutePoint pts[4000];
    int n = buildRoute(10.0f, 10.0f, BAR, OVERLAP, R, pts, 4000);
    assert(n > 50);
    int totalLanes = laneCount(10.0f, BAR, OVERLAP);
    for (int lane = 0; lane < totalLanes; lane++) {
      float laneX = laneCenterX(lane, BAR, OVERLAP);
      bool covered = false;
      for (int i = 0; i < n; i++) {
        if (pts[i].spray && fabsf(pts[i].x - laneX) < 0.05f) { covered = true; break; }
      }
      assert(covered);
    }
    printf("route_test: 10x10 -> %d points, %d lanes\n", n, totalLanes);
  }

  // --- following the route -----------------------------------------------
  // Drive a simulated Ackermann rover along the plan and require it to stay on
  // it. This is the part that used to fail on grass with no way to see why.
  {
    static RoutePoint pts[4000];
    int n = buildRoute(21.91f, 21.91f, BAR, OVERLAP, R, pts, 4000);

    float x = pts[0].x, y = pts[0].y, heading = 0.0f;  // 0 = +Y, clockwise
    const float STEP = 0.15f;      // ft travelled per tick
    const float LOOKAHEAD = 2.5f;
    int idx = 0;
    float worstOffLine = 0.0f;
    int ticks = 0;

    while (idx < n - 1 && ticks < 40000) {
      idx = nearestRouteIndex(pts, n, idx, x, y, 80);
      int la = lookaheadRouteIndex(pts, n, idx, x, y, LOOKAHEAD);

      float dx = pts[la].x - x;
      float dy = pts[la].y - y;
      float want = bearingToWaypointDeg(dx, dy);
      float err = angleDiffDeg(want, heading);

      // A real rover cannot turn tighter than its radius: cap the rate.
      float maxTurnDeg = (STEP / R) * 180.0f / (float)M_PI;
      float turn = err;
      if (turn > maxTurnDeg) turn = maxTurnDeg;
      if (turn < -maxTurnDeg) turn = -maxTurnDeg;

      heading = fmodf(heading + turn + 360.0f, 360.0f);
      float rad = heading * (float)M_PI / 180.0f;
      x += STEP * sinf(rad);
      y += STEP * cosf(rad);

      float offX = pts[idx].x - x, offY = pts[idx].y - y;
      float off = sqrtf(offX * offX + offY * offY);
      if (off > worstOffLine) worstOffLine = off;
      ticks++;
    }

    assert(idx >= n - 2);                 // reached the end of the plan
    assert(worstOffLine < 1.5f);          // never wandered far from it
    printf("route_test: simulated follow finished in %d ticks, worst %.2f ft off\n",
           ticks, worstOffLine);
  }

  printf("route_test: all assertions passed\n");
  return 0;
}
