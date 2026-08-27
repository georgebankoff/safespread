#pragma once
#include <cmath>

enum DubinsWord { D_LSL, D_LSR, D_RSL, D_RSR, D_RLR, D_LRL, D_NONE };

struct DubinsPath {
  DubinsWord word;
  float t, p, q;
  float radius;
  float x0, y0, th0;
};

inline float dubMod2Pi(float angle) {
  const float tau = 6.28318530718f;
  while (angle < 0.0f) angle += tau;
  while (angle >= tau) angle -= tau;
  return angle;
}

inline bool dubinsCompute(float x0, float y0, float th0,
                          float x1, float y1, float th1,
                          float radius, DubinsPath &out) {
  if (radius <= 0.0f) return false;
  const float dx = x1 - x0, dy = y1 - y0;
  const float distance = sqrtf(dx * dx + dy * dy);
  const float d = distance / radius;
  const float theta = distance > 1e-6f ? dubMod2Pi(atan2f(dy, dx)) : 0.0f;
  const float a = dubMod2Pi(th0 - theta);
  const float b = dubMod2Pi(th1 - theta);
  const float sa = sinf(a), ca = cosf(a), sb = sinf(b), cb = cosf(b);
  const float cab = cosf(a - b);
  float best = 1e18f;
  out.word = D_NONE;

  auto consider = [&](DubinsWord word, float t, float p, float q) {
    if (!(t >= 0.0f) || !(p >= 0.0f) || !(q >= 0.0f)) return;
    const float length = t + p + q;
    if (length < best) {
      best = length;
      out.word = word;
      out.t = t; out.p = p; out.q = q;
    }
  };

  {
    const float temp = atan2f(cb - ca, d + sa - sb);
    const float pp = 2.0f + d * d - 2.0f * cab + 2.0f * d * (sa - sb);
    if (pp >= 0.0f) consider(D_LSL, dubMod2Pi(-a + temp), sqrtf(pp), dubMod2Pi(b - temp));
  }
  {
    const float temp = atan2f(ca - cb, d - sa + sb);
    const float pp = 2.0f + d * d - 2.0f * cab + 2.0f * d * (sb - sa);
    if (pp >= 0.0f) consider(D_RSR, dubMod2Pi(a - temp), sqrtf(pp), dubMod2Pi(-b + temp));
  }
  {
    const float pp = -2.0f + d * d + 2.0f * cab + 2.0f * d * (sa + sb);
    if (pp >= 0.0f) {
      const float p = sqrtf(pp);
      const float temp = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0f, p);
      consider(D_LSR, dubMod2Pi(-a + temp), p, dubMod2Pi(-dubMod2Pi(b) + temp));
    }
  }
  {
    const float pp = -2.0f + d * d + 2.0f * cab - 2.0f * d * (sa + sb);
    if (pp >= 0.0f) {
      const float p = sqrtf(pp);
      const float temp = atan2f(ca + cb, d - sa - sb) - atan2f(2.0f, p);
      consider(D_RSL, dubMod2Pi(a - temp), p, dubMod2Pi(b - temp));
    }
  }
  {
    const float temp = (6.0f - d * d + 2.0f * cab + 2.0f * d * (sa - sb)) / 8.0f;
    if (fabsf(temp) <= 1.0f) {
      const float p = dubMod2Pi(6.28318530718f - acosf(temp));
      const float t = dubMod2Pi(a - atan2f(ca - cb, d - sa + sb) + p / 2.0f);
      consider(D_RLR, t, p, dubMod2Pi(a - b - t + p));
    }
  }
  {
    const float temp = (6.0f - d * d + 2.0f * cab + 2.0f * d * (sb - sa)) / 8.0f;
    if (fabsf(temp) <= 1.0f) {
      const float p = dubMod2Pi(6.28318530718f - acosf(temp));
      const float t = dubMod2Pi(-a + atan2f(-ca + cb, d + sa - sb) + p / 2.0f);
      consider(D_LRL, t, p, dubMod2Pi(dubMod2Pi(b) - a - t + p));
    }
  }

  if (out.word == D_NONE) return false;
  out.radius = radius;
  out.x0 = x0; out.y0 = y0; out.th0 = th0;
  return true;
}

inline float dubinsLength(const DubinsPath &path) {
  return (path.t + path.p + path.q) * path.radius;
}

inline void dubinsSegmentTurns(DubinsWord word, int turns[3]) {
  switch (word) {
    case D_LSL: turns[0] = +1; turns[1] = 0;  turns[2] = +1; break;
    case D_LSR: turns[0] = +1; turns[1] = 0;  turns[2] = -1; break;
    case D_RSL: turns[0] = -1; turns[1] = 0;  turns[2] = +1; break;
    case D_RSR: turns[0] = -1; turns[1] = 0;  turns[2] = -1; break;
    case D_RLR: turns[0] = -1; turns[1] = +1; turns[2] = -1; break;
    case D_LRL: turns[0] = +1; turns[1] = -1; turns[2] = +1; break;
    default: turns[0] = turns[1] = turns[2] = 0; break;
  }
}

inline void dubinsPoseAt(const DubinsPath &path, float distance,
                         float &x, float &y, float &heading) {
  int turns[3];
  dubinsSegmentTurns(path.word, turns);
  const float lengths[3] = {path.t, path.p, path.q};
  x = path.x0; y = path.y0; heading = path.th0;
  float remaining = distance / path.radius;
  for (int index = 0; index < 3 && remaining > 1e-6f; ++index) {
    const float segment = lengths[index] < remaining ? lengths[index] : remaining;
    if (turns[index] == 0) {
      x += segment * path.radius * cosf(heading);
      y += segment * path.radius * sinf(heading);
    } else {
      const float direction = static_cast<float>(turns[index]);
      const float nextHeading = heading + direction * segment;
      x += path.radius * direction * (sinf(nextHeading) - sinf(heading));
      y += path.radius * direction * (cosf(heading) - cosf(nextHeading));
      heading = nextHeading;
    }
    remaining -= segment;
  }
}
