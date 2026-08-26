#include <cassert>
#include <cstdio>
#include <cmath>
#include "../throttle.h"

static const float TARGET = 1.8f;
static const float GAIN   = 15.0f;
static const float MINO   = 70.0f;
static const float MAXO   = 400.0f;
static const float START  = 150.0f;

/** A rover that needs `breakawayUs` of throttle before it moves at all, and
 *  then gains `fpsPerUs` of speed for every further microsecond. Loading the
 *  tank raises the breakaway point, which is exactly the case that leaves a
 *  fixed pulse sitting still. */
static float speedFor(float offsetUs, float breakawayUs, float fpsPerUs) {
  if (offsetUs <= breakawayUs) return 0.0f;
  return (offsetUs - breakawayUs) * fpsPerUs;
}

static float settle(float breakawayUs, float fpsPerUs, int steps, float *finalFps) {
  float offset = START, fps = 0.0f;
  for (int i = 0; i < steps; i++) {
    fps = updateSpeedFps(fps, speedFor(offset, breakawayUs, fpsPerUs) * 0.2f, 0.0f,
                         200, 0.4f);
    offset = governThrottle(offset, fps, TARGET, GAIN, MINO, MAXO);
  }
  if (finalFps) *finalFps = fps;
  return offset;
}

int main() {
  // --- speed from position ------------------------------------------------
  {
    // A foot in half a second is two feet per second; the first sample is
    // smoothed in from zero.
    float fps = updateSpeedFps(0.0f, 1.0f, 0.0f, 500, 1.0f);
    assert(fabsf(fps - 2.0f) < 1e-3f);
    // Diagonal motion counts its true distance.
    float d = updateSpeedFps(0.0f, 3.0f, 4.0f, 1000, 1.0f);
    assert(fabsf(d - 5.0f) < 1e-3f);
    // A zero interval must not divide by zero.
    assert(updateSpeedFps(1.23f, 1.0f, 0.0f, 0, 1.0f) == 1.23f);
  }

  // --- an empty rover -----------------------------------------------------
  {
    float fps;
    float off = settle(100.0f, 0.02f, 200, &fps);
    assert(fabsf(fps - TARGET) < 0.1f);
    assert(off > MINO && off < MAXO);
    printf("throttle_test: light rover settles at +%.0fus for %.1f ft/s\n", off, fps);
  }

  // --- the same rover with a full tank ------------------------------------
  // Needs far more throttle to break away. A fixed pulse tuned on the empty
  // one leaves this one stationary; the loop has to find the higher number.
  {
    float fps;
    float off = settle(220.0f, 0.012f, 200, &fps);
    assert(off > 220.0f);                 // actually moving
    assert(fabsf(fps - TARGET) < 0.15f);
    printf("throttle_test: loaded rover settles at +%.0fus for %.1f ft/s\n", off, fps);

    // The fixed pulse this project shipped with would not have moved it.
    assert(speedFor(120.0f, 220.0f, 0.012f) == 0.0f);
  }

  // --- a rover that cannot reach the target -------------------------------
  // Stuck, or grass too deep. It must push to the limit and stop there rather
  // than winding up forever.
  {
    float off = settle(1000.0f, 0.01f, 500, NULL);
    assert(fabsf(off - MAXO) < 0.01f);
    printf("throttle_test: stuck rover pins at +%.0fus and stays there\n", off);
  }

  // --- and one that is already too fast -----------------------------------
  {
    float off = START, fps = 0.0f;
    for (int i = 0; i < 200; i++) {
      fps = updateSpeedFps(fps, speedFor(off, 60.0f, 0.03f) * 0.2f, 0.0f, 200, 0.4f);
      off = governThrottle(off, fps, TARGET, GAIN, MINO, MAXO);
    }
    assert(fabsf(fps - TARGET) < 0.1f);
    assert(off >= MINO);
    printf("throttle_test: fast rover backs down to +%.0fus for %.1f ft/s\n", off, fps);
  }

  // --- never outside its limits, whatever it is fed -----------------------
  {
    for (float fps = -5.0f; fps <= 20.0f; fps += 0.5f) {
      float off = governThrottle(START, fps, TARGET, GAIN, MINO, MAXO);
      assert(off >= MINO && off <= MAXO);
    }
  }

  printf("throttle_test: all assertions passed\n");
  return 0;
}
