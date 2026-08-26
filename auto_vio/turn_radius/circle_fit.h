#pragma once
#include <math.h>

// Least-squares circle through a set of sampled positions (Kasa's algebraic
// fit). Driving at full lock traces a circle; recovering its radius from the
// positions the phone reports is what turns "some turning circle" into the
// number the route planner needs.
//
// The algebraic fit is used rather than a geometric one because it is a single
// 3x3 solve with no iteration -- it cannot fail to converge on a rover, and on
// arcs of 90 degrees or more its bias is far smaller than the spread of the
// samples themselves.

struct CircleFit {
  bool  ok;
  float cx, cy;
  float r;
  float rms;   // RMS distance of the samples from the fitted circle, in ft
  int   n;
};

inline CircleFit fitCircle(const float *xs, const float *ys, int n) {
  CircleFit f;
  f.ok = false; f.cx = 0; f.cy = 0; f.r = 0; f.rms = 0; f.n = n;
  if (n < 3) return f;

  // Work about the centroid: the raw normal equations lose precision badly
  // when the arc sits far from the origin, which it does whenever the rover
  // has driven any distance before the measurement starts.
  double mx = 0, my = 0;
  for (int i = 0; i < n; i++) { mx += xs[i]; my += ys[i]; }
  mx /= n; my /= n;

  double Suu = 0, Suv = 0, Svv = 0;
  double Suuu = 0, Svvv = 0, Suvv = 0, Svuu = 0;
  for (int i = 0; i < n; i++) {
    double u = xs[i] - mx, v = ys[i] - my;
    Suu += u * u;  Suv += u * v;  Svv += v * v;
    Suuu += u * u * u;  Svvv += v * v * v;
    Suvv += u * v * v;  Svuu += v * u * u;
  }

  // Points strung out along a line leave the centre undetermined: any circle
  // large enough passes through them. Report that rather than a huge radius
  // conjured out of noise.
  double det = Suu * Svv - Suv * Suv;
  if (!(Suu > 0.0) || !(Svv > 0.0) || fabs(det) < 1e-6 * Suu * Svv) return f;

  double b1 = 0.5 * (Suuu + Suvv);
  double b2 = 0.5 * (Svvv + Svuu);
  double uc = (b1 * Svv - b2 * Suv) / det;
  double vc = (b2 * Suu - b1 * Suv) / det;

  double r2 = uc * uc + vc * vc + (Suu + Svv) / (double)n;
  if (!(r2 > 0.0)) return f;

  f.cx = (float)(uc + mx);
  f.cy = (float)(vc + my);
  f.r  = (float)sqrt(r2);

  double acc = 0;
  for (int i = 0; i < n; i++) {
    double dx = xs[i] - f.cx, dy = ys[i] - f.cy;
    double e = sqrt(dx * dx + dy * dy) - f.r;
    acc += e * e;
  }
  f.rms = (float)sqrt(acc / (double)n);
  f.ok = true;
  return f;
}

/** Independent estimate from distance travelled against angle swept. It shares
 *  no arithmetic with the fit above, so the two agreeing is real evidence the
 *  measurement is sound rather than a self-consistent mistake. */
inline float arcRadius(float pathLengthFt, float sweptDeg) {
  if (sweptDeg < 1.0f) return 0.0f;
  return pathLengthFt / (sweptDeg * (float)M_PI / 180.0f);
}
