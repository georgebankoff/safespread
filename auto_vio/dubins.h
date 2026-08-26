#pragma once
#include <cmath>
#include <cstddef>

// Shortest path between two poses for a vehicle that cannot turn tighter than
// a given radius. Six candidate words (two arcs with a straight or a third arc
// between them); the shortest valid one wins. This is what makes a planned
// route drivable: every turn it emits respects the rover's turning circle by
// construction, instead of being a target it may or may not be able to reach.
//
// Worked in the usual maths convention -- x east, y north, angles from +X and
// counter-clockwise positive -- and converted at the edges, because the rover's
// own frame measures heading clockwise from +Y.

enum DubinsWord { D_LSL, D_LSR, D_RSL, D_RSR, D_RLR, D_LRL, D_NONE };

struct DubinsPath {
  DubinsWord word;
  float t, p, q;      // segment lengths, in radius-normalised units
  float radius;
  float x0, y0, th0;  // start pose, maths convention
};

inline float dubMod2Pi(float a) {
  const float TAU = 6.28318530718f;
  while (a < 0.0f) a += TAU;
  while (a >= TAU) a -= TAU;
  return a;
}

inline bool dubinsCompute(float x0, float y0, float th0,
                          float x1, float y1, float th1,
                          float radius, DubinsPath &out) {
  if (radius <= 0.0f) return false;

  float dx = x1 - x0, dy = y1 - y0;
  float D = sqrtf(dx * dx + dy * dy);
  float d = D / radius;
  float theta = (D > 1e-6f) ? dubMod2Pi(atan2f(dy, dx)) : 0.0f;
  float a = dubMod2Pi(th0 - theta);
  float b = dubMod2Pi(th1 - theta);

  float sa = sinf(a), ca = cosf(a);
  float sb = sinf(b), cb = cosf(b);
  float cab = cosf(a - b);

  float best = 1e18f;
  out.word = D_NONE;

  auto consider = [&](DubinsWord w, float t, float p, float q) {
    if (!(t >= 0.0f) || !(p >= 0.0f) || !(q >= 0.0f)) return;  // also rejects NaN
    float len = t + p + q;
    if (len < best) {
      best = len;
      out.word = w;
      out.t = t; out.p = p; out.q = q;
    }
  };

  // LSL
  {
    float tmp = atan2f(cb - ca, d + sa - sb);
    float pp = 2.0f + d * d - 2.0f * cab + 2.0f * d * (sa - sb);
    if (pp >= 0.0f) consider(D_LSL, dubMod2Pi(-a + tmp), sqrtf(pp), dubMod2Pi(b - tmp));
  }
  // RSR
  {
    float tmp = atan2f(ca - cb, d - sa + sb);
    float pp = 2.0f + d * d - 2.0f * cab + 2.0f * d * (sb - sa);
    if (pp >= 0.0f) consider(D_RSR, dubMod2Pi(a - tmp), sqrtf(pp), dubMod2Pi(-b + tmp));
  }
  // LSR
  {
    float pp = -2.0f + d * d + 2.0f * cab + 2.0f * d * (sa + sb);
    if (pp >= 0.0f) {
      float p = sqrtf(pp);
      float tmp = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0f, p);
      consider(D_LSR, dubMod2Pi(-a + tmp), p, dubMod2Pi(-dubMod2Pi(b) + tmp));
    }
  }
  // RSL
  {
    float pp = -2.0f + d * d + 2.0f * cab - 2.0f * d * (sa + sb);
    if (pp >= 0.0f) {
      float p = sqrtf(pp);
      float tmp = atan2f(ca + cb, d - sa - sb) - atan2f(2.0f, p);
      consider(D_RSL, dubMod2Pi(a - tmp), p, dubMod2Pi(b - tmp));
    }
  }
  // RLR
  {
    float tmp = (6.0f - d * d + 2.0f * cab + 2.0f * d * (sa - sb)) / 8.0f;
    if (fabsf(tmp) <= 1.0f) {
      float p = dubMod2Pi(6.28318530718f - acosf(tmp));
      float t = dubMod2Pi(a - atan2f(ca - cb, d - sa + sb) + p / 2.0f);
      consider(D_RLR, t, p, dubMod2Pi(a - b - t + p));
    }
  }
  // LRL
  {
    float tmp = (6.0f - d * d + 2.0f * cab + 2.0f * d * (sb - sa)) / 8.0f;
    if (fabsf(tmp) <= 1.0f) {
      float p = dubMod2Pi(6.28318530718f - acosf(tmp));
      float t = dubMod2Pi(-a + atan2f(-ca + cb, d + sa - sb) + p / 2.0f);
      consider(D_LRL, t, p, dubMod2Pi(dubMod2Pi(b) - a - t + p));
    }
  }

  if (out.word == D_NONE) return false;
  out.radius = radius;
  out.x0 = x0; out.y0 = y0; out.th0 = th0;
  return true;
}

inline float dubinsLength(const DubinsPath &p) {
  return (p.t + p.p + p.q) * p.radius;
}

// Which way each of the three segments bends: +1 left, 0 straight, -1 right.
inline void dubinsSegmentTurns(DubinsWord w, int turns[3]) {
  switch (w) {
    case D_LSL: turns[0] = +1; turns[1] = 0;  turns[2] = +1; break;
    case D_LSR: turns[0] = +1; turns[1] = 0;  turns[2] = -1; break;
    case D_RSL: turns[0] = -1; turns[1] = 0;  turns[2] = +1; break;
    case D_RSR: turns[0] = -1; turns[1] = 0;  turns[2] = -1; break;
    case D_RLR: turns[0] = -1; turns[1] = +1; turns[2] = -1; break;
    case D_LRL: turns[0] = +1; turns[1] = -1; turns[2] = +1; break;
    default:    turns[0] = turns[1] = turns[2] = 0; break;
  }
}

// Pose a given arc-length along the path. Integrating in closed form keeps the
// sampled points exactly on the arcs, so tracking error is the rover's, not the
// plan's.
inline void dubinsPoseAt(const DubinsPath &path, float s,
                         float &x, float &y, float &th) {
  int turns[3];
  dubinsSegmentTurns(path.word, turns);
  float lens[3] = { path.t, path.p, path.q };

  x = path.x0; y = path.y0; th = path.th0;
  float remaining = s / path.radius;  // normalised

  for (int i = 0; i < 3; i++) {
    float seg = lens[i] < remaining ? lens[i] : remaining;
    if (seg <= 0.0f) { if (remaining <= 0.0f) break; else continue; }

    if (turns[i] == 0) {
      x += seg * path.radius * cosf(th);
      y += seg * path.radius * sinf(th);
    } else {
      float dir = (float)turns[i];
      float thNew = th + dir * seg;
      // Exact arc endpoint: centre is one radius to the turning side.
      x += path.radius * (dir * (sinf(thNew) - sinf(th))) * (dir > 0 ? 1.0f : 1.0f);
      y += path.radius * (dir * (cosf(th) - cosf(thNew))) * (dir > 0 ? 1.0f : 1.0f);
      th = thNew;
    }

    remaining -= seg;
    if (remaining <= 1e-6f) break;
  }
}
