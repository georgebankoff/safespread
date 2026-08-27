#include <cassert>
#include <cmath>
#include <cstdio>
#include "../speed_control.h"

int main() {
  assert(DEFAULT_STRAIGHT_SPEED_FPS == 1.0f);
  assert(DEFAULT_TURN_SPEED_FPS == 0.7f);

  SpeedPI atSpeed;
  atSpeed.feedForwardUs = 120.0f;
  assert(atSpeed.update(1.0f, 1.0f, 0.05f) == 120);
  assert(std::fabs(atSpeed.integralUs) < 0.001f);

  // Reverse uses the same learned magnitude with a signed target/output.
  SpeedPI reverse;
  reverse.feedForwardUs = 125.0f;
  assert(reverse.update(-1.0f, -1.0f, 0.05f) == -125);

  // PI correction is bounded independently, then the complete ESC offset is
  // clamped to the absolute 350 us hardware envelope.
  SpeedPI bounded;
  bounded.feedForwardUs = 200.0f;
  assert(bounded.update(10.0f, 0.1f, 0.05f) == 350);
  assert(std::fabs(bounded.lastCorrectionUs) <= 250.0f);

  // Saturation does not wind the integrator farther into either rail.
  SpeedPI upper;
  upper.feedForwardUs = 300.0f;
  upper.integralUs = 100.0f;
  upper.update(1.0f, 0.5f, 0.5f);
  assert(std::fabs(upper.integralUs - 100.0f) < 0.001f);

  SpeedPI lower;
  lower.feedForwardUs = 300.0f;
  lower.integralUs = -100.0f;
  lower.update(-1.0f, -0.5f, 0.5f);
  assert(std::fabs(lower.integralUs + 100.0f) < 0.001f);

  // With no measured motion, use a bounded breakaway ramp rather than
  // integrating an ever-growing throttle request.
  SpeedPI breakaway;
  breakaway.feedForwardUs = 120.0f;
  int first = breakaway.update(1.0f, 0.0f, 0.2f);
  int second = breakaway.update(1.0f, 0.0f, 0.2f);
  assert(first > 120 && second > first);
  assert(std::fabs(breakaway.integralUs) < 0.001f);

  // One second without motion faults and returns neutral offset. Further
  // calls cannot wind throttle until an explicit reset.
  SpeedPI noMotion;
  noMotion.feedForwardUs = 120.0f;
  for (int index = 0; index < 5; ++index) noMotion.update(1.0f, 0.0f, 0.2f);
  assert(noMotion.stalled);
  assert(noMotion.lastOffsetUs == 0);
  const float frozenIntegral = noMotion.integralUs;
  assert(noMotion.update(1.0f, 0.0f, 0.2f) == 0);
  assert(noMotion.integralUs == frozenIntegral);
  noMotion.reset();
  assert(!noMotion.stalled && noMotion.lastOffsetUs == 0);

  // Invalid timing samples are ignored without modifying controller state.
  SpeedPI badDt;
  badDt.feedForwardUs = 120.0f;
  int prior = badDt.update(1.0f, 0.8f, 0.1f);
  const float priorIntegral = badDt.integralUs;
  assert(badDt.update(1.0f, 0.8f, 0.0f) == prior);
  assert(badDt.update(1.0f, 0.8f, 1.0f) == prior);
  assert(std::isnan(0.0f / 0.0f));
  assert(badDt.update(1.0f, 0.8f, 0.0f / 0.0f) == prior);
  assert(badDt.integralUs == priorIntegral);

  std::printf("speed_control_test: all assertions passed\n");
  return 0;
}
