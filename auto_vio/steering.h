#pragma once
#include <math.h>

// Where "straight ahead" actually is, and how a steering command becomes a
// servo pulse.
//
// The servo's mechanical mid-travel is not the same thing as the wheels
// pointing straight. If the two are assumed equal when they are not, the rover
// settles a fixed distance to one side of whatever line it is tracking -- and
// because that offset is fixed in the rover's own frame, it flips sides in the
// field every time the rover turns around. Outbound passes land left of plan,
// return passes land right of plan, and neighbouring lanes collapse onto each
// other in pairs with gaps between them.

/** The pulse at which the wheels really point straight, derived from the two
 *  measured turning circles. Steering angle grows with distance from the true
 *  centre and turning radius goes as the reciprocal of the angle, so
 *      (leftLock - c) / (c - rightLock) = rRight / rLeft
 *  which solves directly for c. Equal radii give the midpoint, as they should. */
inline float steeringCentreUs(float rLeftFt, float rRightFt,
                              float leftLockUs, float rightLockUs) {
  if (!(rLeftFt > 0.0f) || !(rRightFt > 0.0f)) {
    return (leftLockUs + rightLockUs) * 0.5f;
  }
  float k = rRightFt / rLeftFt;
  return (leftLockUs + k * rightLockUs) / (1.0f + k);
}

/** Command to pulse. `rightward` > 0 steers toward increasing heading, with
 *  `maxOffset` meaning full lock. Each side is scaled by its own travel from
 *  the centre, which is not the same on both sides, so an equal command asks
 *  for an equal fraction of the turn available that way. */
inline float steerPulseUs(float rightward, float centreUs,
                          float leftLockUs, float rightLockUs, float maxOffset) {
  float frac = fabsf(rightward) / maxOffset;
  if (frac > 1.0f) frac = 1.0f;

  float us = (rightward >= 0.0f) ? centreUs - frac * (centreUs - rightLockUs)
                                 : centreUs + frac * (leftLockUs - centreUs);
  if (us < rightLockUs) us = rightLockUs;
  if (us > leftLockUs)  us = leftLockUs;
  return us;
}

/** One step of learning the centre. On a straight pass the steering has to
 *  average out to whatever pulse really does point the wheels straight, so the
 *  average of what gets applied IS the centre. Learning it absorbs whatever
 *  the measured radii did not capture -- a servo that has crept, tyre pull, a
 *  cambered lawn -- and a constant bias is precisely the error that produces
 *  paired passes rather than harmless wander. */
inline float updateSteeringTrim(float trimUs, float appliedUs, float centreUs,
                                float rate, float limitUs) {
  trimUs += rate * (appliedUs - centreUs);
  if (trimUs > limitUs)  trimUs = limitUs;
  if (trimUs < -limitUs) trimUs = -limitUs;
  return trimUs;
}
