#include <cassert>
#include <cstdio>
#include <cmath>
#include "../dubins.h"

static float angDiff(float a, float b) {
  float d = a - b;
  while (d > M_PI) d -= 2 * M_PI;
  while (d < -M_PI) d += 2 * M_PI;
  return d;
}

// The real check: walk the path and confirm it actually arrives at the goal.
// A wrong closed-form formula shows up here rather than on grass.
static void checkReaches(float x0, float y0, float th0,
                         float x1, float y1, float th1,
                         float R, const char *label) {
  DubinsPath p;
  assert(dubinsCompute(x0, y0, th0, x1, y1, th1, R, p));

  float len = dubinsLength(p);
  assert(len > 0.0f && len < 1000.0f);

  float ex, ey, eth;
  dubinsPoseAt(p, len, ex, ey, eth);

  float posErr = sqrtf((ex - x1) * (ex - x1) + (ey - y1) * (ey - y1));
  float headErr = fabsf(angDiff(eth, th1));

  if (posErr > 0.02f || headErr > 0.02f) {
    printf("FAIL %s: word=%d len=%.2f end=(%.3f,%.3f,%.3f) want=(%.3f,%.3f,%.3f)\n",
           label, (int)p.word, len, ex, ey, eth, x1, y1, th1);
  }
  assert(posErr <= 0.02f);
  assert(headErr <= 0.02f);
}

// No sampled step may imply a turn tighter than the rover can drive.
static void checkCurvature(float x0, float y0, float th0,
                           float x1, float y1, float th1, float R) {
  DubinsPath p;
  assert(dubinsCompute(x0, y0, th0, x1, y1, th1, R, p));
  float len = dubinsLength(p);

  const float step = 0.05f;
  float px, py, pth;
  dubinsPoseAt(p, 0.0f, px, py, pth);
  for (float s = step; s <= len; s += step) {
    float x, y, th;
    dubinsPoseAt(p, s, x, y, th);
    float moved = sqrtf((x - px) * (x - px) + (y - py) * (y - py));
    float turned = fabsf(angDiff(th, pth));
    if (moved > 1e-4f) {
      float curvature = turned / moved;
      assert(curvature <= (1.0f / R) + 0.05f);
    }
    px = x; py = y; pth = th;
  }
}

int main() {
  const float R = 4.0f;
  const float PI = (float)M_PI;

  // A straight run ahead.
  checkReaches(0, 0, 0, 10, 0, 0, R, "straight");

  // The case that matters: reverse direction, shifted sideways by a full
  // turning diameter -- the lane-to-lane transition after a skip.
  checkReaches(0, 0, PI / 2, 2 * R, 0, -PI / 2, R, "u-turn 2R over");

  // Shifted much further than a diameter: arc, straight, arc.
  checkReaches(0, 0, PI / 2, 20, 0, -PI / 2, R, "u-turn wide");

  // Shifted less than a diameter, which needs a three-arc word rather than
  // a simple half circle -- the case a plain sweep cannot do.
  checkReaches(0, 0, PI / 2, 1.0f, 0, -PI / 2, R, "u-turn tight");
  checkReaches(0, 0, PI / 2, 0.0f, 0, -PI / 2, R, "u-turn in place");

  // Assorted poses, including backwards and diagonal goals.
  checkReaches(0, 0, 0, -6, 3, PI, R, "behind left");
  checkReaches(0, 0, 0, -6, -3, PI / 2, R, "behind right");
  checkReaches(2, -3, 1.0f, -5, 7, -2.0f, R, "arbitrary");

  // Every emitted path must be drivable by this rover.
  checkCurvature(0, 0, PI / 2, 2 * R, 0, -PI / 2, R);
  checkCurvature(0, 0, PI / 2, 1.0f, 0, -PI / 2, R);
  checkCurvature(2, -3, 1.0f, -5, 7, -2.0f, R);

  // Sweep a grid of goal poses. A closed-form slip in a word that the handful
  // of cases above happen not to select shows up here -- which is exactly how
  // the LRL formula was found to be wrong.
  {
    bool wordSeen[D_NONE] = {false, false, false, false, false, false};
    int checked = 0;
    for (float gx = -12.0f; gx <= 12.0f; gx += 1.5f) {
      for (float gy = -12.0f; gy <= 12.0f; gy += 1.5f) {
        for (int k = 0; k < 8; k++) {
          float gth = k * PI / 4.0f;
          DubinsPath p;
          if (!dubinsCompute(0, 0, 0, gx, gy, gth, R, p)) continue;
          if (p.word < D_NONE) wordSeen[p.word] = true;

          float len = dubinsLength(p);
          float ex, ey, eth;
          dubinsPoseAt(p, len, ex, ey, eth);
          float posErr = sqrtf((ex - gx) * (ex - gx) + (ey - gy) * (ey - gy));
          float headErr = fabsf(angDiff(eth, gth));
          if (posErr > 0.02f || headErr > 0.02f) {
            printf("FAIL sweep word=%d goal=(%.1f,%.1f,%.2f) got=(%.2f,%.2f,%.2f)\n",
                   (int)p.word, gx, gy, gth, ex, ey, eth);
          }
          assert(posErr <= 0.02f);
          assert(headErr <= 0.02f);
          checked++;
        }
      }
    }
    // All six words must actually have been produced, or the sweep proves less
    // than it appears to.
    for (int w = 0; w < D_NONE; w++) {
      if (!wordSeen[w]) printf("word %d never exercised\n", w);
      assert(wordSeen[w]);
    }
    printf("dubins_test: swept %d poses, all six words exercised\n", checked);
  }

  // A tight shift should cost more travel than a clean one, since the rover
  // has to loop out and come back.
  DubinsPath easy, tight;
  dubinsCompute(0, 0, PI / 2, 2 * R, 0, -PI / 2, R, easy);
  dubinsCompute(0, 0, PI / 2, 0.5f, 0, -PI / 2, R, tight);
  assert(dubinsLength(tight) > dubinsLength(easy));

  printf("dubins_test: all assertions passed (2R u-turn %.1f ft, tight %.1f ft)\n",
         dubinsLength(easy), dubinsLength(tight));
  return 0;
}
