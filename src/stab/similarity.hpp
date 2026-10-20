#pragma once

namespace stab {

// A 2D similarity transform in centre-of-frame coordinates, stored as
// parameters so camera paths can be accumulated and smoothed component-wise.
struct Similarity {
  double dx = 0;        // pixels
  double dy = 0;        // pixels
  double angle = 0;     // radians
  double logScale = 0;  // log of uniform scale

  Similarity operator+(const Similarity& o) const {
    return {dx + o.dx, dy + o.dy, angle + o.angle, logScale + o.logScale};
  }
  Similarity operator-(const Similarity& o) const {
    return {dx - o.dx, dy - o.dy, angle - o.angle, logScale - o.logScale};
  }
  Similarity operator*(double k) const { return {dx * k, dy * k, angle * k, logScale * k}; }
};

}  // namespace stab
