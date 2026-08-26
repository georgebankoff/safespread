#include <cassert>
#include <cstdio>
#include <cmath>
#include "../route.h"

static const float BAR     = 17.0f / 12.0f;
static const float OVERLAP = 0.15f;
// The rover's measured circles, which are not the same size.
static const float RL = 4.33f;
static const float RR = 2.92f;

static RoutePoint pts[6000];
static RoutePoint sim[6000];

// Drive a simulated Ackermann rover, with the rover's real asymmetric radii,
// along the whole plan -- reversing legs included. This is the part that used
// to fail on grass with no way to see why.
static void simulateFollow(float field, const char *label) {
  const float STEP           = 0.15f;   // ft per tick
  const float LOOKAHEAD      = 2.5f;    // straight passes
  const float LOOKAHEAD_TURN = 1.0f;    // shorter than the tightest radius
  const float CUSP_TOL       = 0.5f;
  const int   WINDOW         = 80;
  const float SPRAY_OFF      = 1.5f;
  const float SPRAY_OFF_1ST  = 3.0f;

  int n = buildRoute(field, field, BAR, OVERLAP, RL, RR, sim, 6000);
  int lanes = laneCount(field, BAR, OVERLAP);
  assert(n > 20);

  int firstPassEnd = n;
  for (int i = 0; i < n; i++) if (!sim[i].spray) { firstPassEnd = i; break; }

  float x = sim[0].x, y = sim[0].y, heading = 0.0f;
  int idx = 0, ticks = 0, stuck = 0;
  float worstOff = 0.0f, worstStraight = 0.0f;
  bool firstPassGap = false;
  float sprayedFt = 0.0f;

  while (idx < n - 1 && ticks < 80000) {
    int prevIdx = idx;
    idx = advanceRouteIndex(sim, n, idx, x, y, WINDOW, CUSP_TOL);
    stuck = (idx == prevIdx) ? stuck + 1 : 0;
    assert(stuck < 400);            // never wedged against a cusp

    bool rev = sim[idx].reverse;
    int la = lookaheadWithinSegment(sim, n, idx, x, y,
                                    sim[idx].turning ? LOOKAHEAD_TURN : LOOKAHEAD);

    float want = bearingToWaypointDeg(sim[la].x - x, sim[la].y - y);
    float reference = rev ? fmodf(heading + 180.0f, 360.0f) : heading;
    float err = angleDiffDeg(want, reference);

    // Which circle the rover is on depends on which way the wheels go, and
    // reversing swaps that: backing up, the nose swings the other way.
    float R = rev ? ((err > 0.0f) ? RL : RR) : ((err > 0.0f) ? RR : RL);

    float maxTurn = (STEP / R) * 180.0f / (float)M_PI;
    float turn = err;
    if (turn > maxTurn) turn = maxTurn;
    if (turn < -maxTurn) turn = -maxTurn;

    heading = fmodf(heading + turn + 360.0f, 360.0f);
    float rad = heading * (float)M_PI / 180.0f;
    float sgn = rev ? -1.0f : 1.0f;
    x += sgn * STEP * sinf(rad);
    y += sgn * STEP * cosf(rad);

    float offX = sim[idx].x - x, offY = sim[idx].y - y;
    float off = sqrtf(offX * offX + offY * offY);
    if (off > worstOff) worstOff = off;
    if (!sim[idx].turning && off > worstStraight) worstStraight = off;

    float limit = (idx < firstPassEnd) ? SPRAY_OFF_1ST : SPRAY_OFF;
    bool spraying = sim[idx].spray && off <= limit;
    if (spraying) sprayedFt += STEP;

    // The first pass must spray without interruption along its length.
    if (idx < firstPassEnd && !spraying && y > 0.5f && y < field - 0.5f) {
      firstPassGap = true;
    }
    ticks++;
  }

  assert(idx >= n - 2);          // reached the end of the plan
  assert(worstOff < 1.5f);       // never wandered far from it
  assert(!firstPassGap);         // first pass sprayed end to end

  // Entering each pass, the rover must be inside a lane width of the line, or
  // it sprays the wrong strip.
  assert(worstStraight < laneSpacing(BAR, OVERLAP));

  float wanted = lanes * field;
  assert(sprayedFt > 0.92f * wanted);

  printf("route_test: %s follow -> %d ticks, worst %.2f ft off "
         "(%.2f on passes), sprayed %.0f/%.0f ft\n",
         label, ticks, worstOff, worstStraight, sprayedFt, wanted);
}

int main() {
  // --- lane geometry ------------------------------------------------------
  {
    // The first lane sits under the rover, so placing the rover is how the
    // first pass gets aimed.
    assert(fabsf(laneCenterX(0, BAR, OVERLAP)) < 0.001f);

    assert(laneCount(10.0f, 2.0f, 0.0f) == 5);
    assert(fabsf(laneCenterX(4, 2.0f, 0.0f) - 8.0f) < 0.001f);

    // Lanes stop before the far edge: a width not divisible by the bar is
    // under-covered at the far side, never overrun.
    int m = laneCount(9.0f, 2.0f, 0.0f);
    float band = (laneCenterX(m - 1, 2.0f, 0.0f) + 1.0f) - (laneCenterX(0, 2.0f, 0.0f) - 1.0f);
    assert(band <= 9.0f + 0.001f);

    assert(laneCount(1.0f, 2.0f, 0.0f) == 1);
    printf("route_test: lane geometry ok (10ft/2ft bar = 5 lanes, real = %d)\n",
           laneCount(21.91f, BAR, OVERLAP));
  }

  // --- the real field -----------------------------------------------------
  int n = 0, lanes = 0;
  const float FIELD = 21.91f;
  {
    n = buildRoute(FIELD, FIELD, BAR, OVERLAP, RL, RR, pts, 6000);
    lanes = laneCount(FIELD, BAR, OVERLAP);
    assert(n > 100 && n < 6000);

    // Every lane must actually get sprayed somewhere along its length.
    for (int lane = 0; lane < lanes; lane++) {
      float laneX = laneCenterX(lane, BAR, OVERLAP);
      bool covered = false;
      for (int i = 0; i < n; i++) {
        if (pts[i].spray && fabsf(pts[i].x - laneX) < 0.05f) { covered = true; break; }
      }
      assert(covered);
    }

    // Spray must never be commanded outside the rectangle, and never while
    // backing up.
    for (int i = 0; i < n; i++) {
      if (!pts[i].spray) continue;
      assert(pts[i].y >= -0.01f && pts[i].y <= FIELD + 0.01f);
      assert(pts[i].x >= -0.01f && pts[i].x <= FIELD + 0.01f);
      assert(!pts[i].reverse);
    }

    // Consecutive points stay close enough to follow.
    for (int i = 1; i < n; i++) {
      float dx = pts[i].x - pts[i - 1].x, dy = pts[i].y - pts[i - 1].y;
      assert(sqrtf(dx * dx + dy * dy) <= ROUTE_STEP_FT * 2.5f);
    }

    // The route begins under the rover, already spraying.
    assert(fabsf(pts[0].x) < 0.01f && fabsf(pts[0].y) < 0.01f);
    assert(pts[0].spray && !pts[0].reverse);

    int reversals = 0;
    for (int i = 1; i < n; i++) if (pts[i].reverse != pts[i - 1].reverse) reversals++;
    assert(reversals > 0);   // it really is using three-point turns
    printf("route_test: real field -> %d points, %d lanes, %d direction changes\n",
           n, lanes, reversals);
  }

  // --- the first pass sprays from end to end ------------------------------
  // This is the pass the user cares most about, and the one most at risk:
  // it sits on x=0, so any test based on being inside the rectangle clips it.
  {
    int firstPassEnd = n;
    for (int i = 0; i < n; i++) if (!pts[i].spray) { firstPassEnd = i; break; }

    assert(firstPassEnd > 2);
    for (int i = 0; i < firstPassEnd; i++) {
      assert(pts[i].spray);
      assert(fabsf(pts[i].x) < 0.01f);          // dead straight
    }
    // and it runs the full length of the field
    assert(fabsf(pts[0].y) < 0.01f);
    assert(fabsf(pts[firstPassEnd - 1].y - FIELD) < 0.01f);
    printf("route_test: first pass sprays %d points, 0.0 -> %.2f ft, unbroken\n",
           firstPassEnd, pts[firstPassEnd - 1].y);
  }

  // --- how far outside the rectangle the route reaches --------------------
  {
    float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
    for (int i = 0; i < n; i++) {
      if (pts[i].x < minX) minX = pts[i].x;
      if (pts[i].x > maxX) maxX = pts[i].x;
      if (pts[i].y < minY) minY = pts[i].y;
      if (pts[i].y > maxY) maxY = pts[i].y;
    }
    // Turns must not swing wildly off the plot; anything much beyond a
    // margin plus a turning radius would need clearance the user may not have.
    assert(maxY <= FIELD + HEADLAND_MARGIN_FT + RL + 0.5f);
    assert(minY >= -(HEADLAND_MARGIN_FT + RL + 0.5f));
    printf("route_test: route spans x %.1f..%.1f, y %.1f..%.1f "
           "(needs %.1f ft clear beyond each end)\n",
           minX, maxX, minY, maxY, maxY - FIELD);
  }

  // --- a small plot still produces a complete route -----------------------
  {
    static RoutePoint small[6000];
    int sn = buildRoute(10.0f, 10.0f, BAR, OVERLAP, RL, RR, small, 6000);
    assert(sn > 50);
    int sl = laneCount(10.0f, BAR, OVERLAP);
    for (int lane = 0; lane < sl; lane++) {
      float laneX = laneCenterX(lane, BAR, OVERLAP);
      bool covered = false;
      for (int i = 0; i < sn; i++) {
        if (small[i].spray && fabsf(small[i].x - laneX) < 0.05f) { covered = true; break; }
      }
      assert(covered);
    }
    printf("route_test: 10x10 -> %d points, %d lanes\n", sn, sl);
  }

  // --- following the route, reversing included ----------------------------
  simulateFollow(FIELD, "real field");
  simulateFollow(10.0f, "10x10");
  simulateFollow(6.0f, "6x6");

  printf("route_test: all assertions passed\n");
  return 0;
}
