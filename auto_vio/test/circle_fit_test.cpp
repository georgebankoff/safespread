#include <cassert>
#include <cstdio>
#include <cmath>
#include "../turn_radius/circle_fit.h"

// Deterministic pseudo-noise so a failure is reproducible rather than a
// once-in-a-while surprise on someone else's machine.
struct Noise {
  unsigned s = 12345u;
  float next(float amplitude) {
    s = s * 1103515245u + 12345u;
    float u = (float)((s >> 16) & 0x7FFF) / 32767.0f;  // 0..1
    return (u * 2.0f - 1.0f) * amplitude;
  }
};

// Sample an arc the way the rover does: centre, radius, and how much of the
// circle it actually got round before the leg ended.
static int makeArc(float cx, float cy, float r, float startDeg, float sweepDeg,
                   int n, float noiseFt, float *xs, float *ys) {
  Noise noise;
  for (int i = 0; i < n; i++) {
    float t = (startDeg + sweepDeg * (float)i / (float)(n - 1)) * (float)M_PI / 180.0f;
    xs[i] = cx + r * cosf(t) + noise.next(noiseFt);
    ys[i] = cy + r * sinf(t) + noise.next(noiseFt);
  }
  return n;
}

int main() {
  float xs[400], ys[400];

  // --- a clean full circle -----------------------------------------------
  {
    int n = makeArc(0, 0, 4.0f, 0, 360, 40, 0.0f, xs, ys);
    CircleFit f = fitCircle(xs, ys, n);
    assert(f.ok);
    assert(fabsf(f.r - 4.0f) < 0.01f);
    assert(f.rms < 0.005f);
  }

  // --- a circle away from the origin, as it will be in the field ---------
  {
    int n = makeArc(37.0f, -19.0f, 6.25f, 0, 360, 60, 0.0f, xs, ys);
    CircleFit f = fitCircle(xs, ys, n);
    assert(f.ok);
    assert(fabsf(f.r - 6.25f) < 0.01f);
    assert(fabsf(f.cx - 37.0f) < 0.01f);
    assert(fabsf(f.cy + 19.0f) < 0.01f);
  }

  // --- a realistic leg: partial arc, noisy positions ----------------------
  // ARKit is good to a couple of inches; the leg may stop short of a full
  // circle. The recovered radius still has to be usable for planning.
  {
    int n = makeArc(2.0f, 5.0f, 4.5f, 20.0f, 200.0f, 45, 0.08f, xs, ys);
    CircleFit f = fitCircle(xs, ys, n);
    assert(f.ok);
    if (fabsf(f.r - 4.5f) >= 0.25f) printf("FAIL noisy arc: r=%.3f\n", f.r);
    assert(fabsf(f.r - 4.5f) < 0.25f);
    printf("circle_fit_test: 200deg arc + 1in noise -> r=%.2f (true 4.50), rms=%.3f\n",
           f.r, f.rms);
  }

  // A short arc is where an algebraic fit is weakest, so state the limit the
  // sketch relies on rather than assuming it.
  {
    int n = makeArc(0, 0, 4.0f, 0.0f, 90.0f, 25, 0.05f, xs, ys);
    CircleFit f = fitCircle(xs, ys, n);
    assert(f.ok);
    if (fabsf(f.r - 4.0f) >= 0.6f) printf("FAIL 90deg arc: r=%.3f\n", f.r);
    assert(fabsf(f.r - 4.0f) < 0.6f);
    printf("circle_fit_test: 90deg arc + 0.6in noise -> r=%.2f (true 4.00)\n", f.r);
  }

  // --- things that must be refused rather than answered wrongly ----------
  {
    // Driving straight: no circle to find.
    for (int i = 0; i < 30; i++) { xs[i] = 0.4f * i; ys[i] = 0.0f; }
    assert(!fitCircle(xs, ys, 30).ok);

    // A diagonal straight line, which a naive degeneracy test misses.
    for (int i = 0; i < 30; i++) { xs[i] = 0.3f * i; ys[i] = 0.3f * i; }
    assert(!fitCircle(xs, ys, 30).ok);

    // Not enough samples to define anything.
    assert(!fitCircle(xs, ys, 2).ok);
  }

  // --- the independent cross-check ---------------------------------------
  {
    // A full circle of radius 4 is 2*pi*4 = 25.13 ft of travel.
    assert(fabsf(arcRadius(25.1327f, 360.0f) - 4.0f) < 0.01f);
    assert(fabsf(arcRadius(12.5664f, 180.0f) - 4.0f) < 0.01f);
    // Barely moved: refuse rather than divide by almost nothing.
    assert(arcRadius(0.2f, 0.3f) == 0.0f);
  }

  // --- end to end, as the sketch will actually run it ---------------------
  // A rover at full lock, poses arriving at 10Hz, positions noisy by a couple
  // of inches, recorded only when it has moved SAMPLE_SPACING_FT. This is the
  // question that decides whether the measurement is worth driving: do those
  // specific choices recover the radius closely enough to plan turns with?
  {
    const float SAMPLE_SPACING_FT = 0.25f;   // must match turn_radius.ino
    const int   MAX_SAMPLES_SKETCH = 400;
    const float TARGET_SWEEP_DEG  = 350.0f;

    for (float trueR = 2.5f; trueR <= 12.0f; trueR += 0.5f) {
      for (float speed = 1.0f; speed <= 4.0f; speed += 1.0f) {
        Noise noise;
        float x = trueR, y = 0.0f;      // start on the circle, heading +Y
        float ang = 0.0f;               // position angle about the centre
        float lastX = x, lastY = y;
        float swept = 0.0f, pathLen = 0.0f;
        int n = 0;

        // 10Hz poses, each advancing the rover along the arc.
        for (int tick = 0; tick < 2000 && n < MAX_SAMPLES_SKETCH &&
                           swept < TARGET_SWEEP_DEG; tick++) {
          float ds = speed * 0.1f;
          ang += ds / trueR;
          swept = ang * 180.0f / (float)M_PI;
          float px = trueR * cosf(ang) + noise.next(0.06f);
          float py = trueR * sinf(ang) + noise.next(0.06f);

          float dx = px - lastX, dy = py - lastY;
          float moved = sqrtf(dx * dx + dy * dy);
          if (moved < SAMPLE_SPACING_FT) continue;
          xs[n] = px; ys[n] = py; n++;
          pathLen += moved;
          lastX = px; lastY = py;
        }

        CircleFit f = fitCircle(xs, ys, n);
        float arc = arcRadius(pathLen, swept);
        assert(f.ok);

        // 10% is comfortably tight enough: the lane skip is rounded to whole
        // lanes, so a tenth of a radius does not change the planned route.
        float err = fabsf(f.r - trueR) / trueR;
        if (err >= 0.10f) {
          printf("FAIL end-to-end: trueR=%.1f speed=%.0f -> fit=%.2f arc=%.2f n=%d\n",
                 trueR, speed, f.r, arc, n);
        }
        assert(err < 0.10f);
        assert(fabsf(arc - trueR) / trueR < 0.10f);
        assert(n >= 20);   // enough points to fit at all
      }
    }
    printf("circle_fit_test: end-to-end sweep (R 2.5-12ft, 1-4 ft/s) within 10%%\n");
  }

  printf("circle_fit_test: all assertions passed\n");
  return 0;
}
