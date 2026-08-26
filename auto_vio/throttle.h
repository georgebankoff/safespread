#pragma once

// Hold a ground speed rather than a throttle position.
//
// A fixed pulse is a guess about the whole vehicle: how heavy the tank is, how
// long the grass is, which way the ground slopes, how charged the battery is.
// A pulse that just moves an empty rover will not move a full one, and one set
// high enough for a full tank runs an empty one too fast. The phone already
// measures ground speed, so the throttle can simply be servoed to it.
//
// This also matters for the route: turning radius depends on speed, and the
// radii the planner is built on were measured at a particular one. Holding
// that same speed is what keeps the measurement valid.

/** One step of the speed loop. Returns the new offset from neutral, in
 *  microseconds -- always positive; the caller applies it above neutral to go
 *  forward or below to reverse. Clamped at both ends, so a stuck rover pushes
 *  to the limit and stops there rather than winding up. */
inline float governThrottle(float offsetUs, float measuredFps, float targetFps,
                            float gainUsPerFps, float minOffsetUs,
                            float maxOffsetUs) {
  offsetUs += gainUsPerFps * (targetFps - measuredFps);
  if (offsetUs < minOffsetUs) offsetUs = minOffsetUs;
  if (offsetUs > maxOffsetUs) offsetUs = maxOffsetUs;
  return offsetUs;
}

/** Smoothed ground speed from successive positions. Returns feet per second.
 *  `dtMs` is the interval those two positions were taken over. */
inline float updateSpeedFps(float previousFps, float dx, float dy,
                            unsigned long dtMs, float smoothing) {
  if (dtMs == 0) return previousFps;
  float sample = sqrtf(dx * dx + dy * dy) / (dtMs / 1000.0f);
  return previousFps + smoothing * (sample - previousFps);
}
