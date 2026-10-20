#include "stab/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace stab {

Similarity smoothAt(const std::vector<Similarity>& path, size_t i, int radius) {
  if (radius <= 0 || path.empty()) return path.at(i);

  const double sigma = radius / 2.0;
  const auto last = static_cast<long>(path.size()) - 1;
  Similarity acc;
  double weight = 0;
  for (int k = -radius; k <= radius; ++k) {
    const double w = std::exp(-(k * k) / (2 * sigma * sigma));
    const long j = std::clamp(static_cast<long>(i) + k, 0L, last);
    acc = acc + path[j] * w;
    weight += w;
  }
  return acc * (1.0 / weight);
}

std::vector<Similarity> smoothGaussian(const std::vector<Similarity>& path, int radius) {
  std::vector<Similarity> out(path.size());
  for (size_t i = 0; i < path.size(); ++i) out[i] = smoothAt(path, i, radius);
  return out;
}

cv::Mat correctionMatrix(const Similarity& c, cv::Size frame, double zoom) {
  const double s = zoom * std::exp(c.logScale);
  const double a = s * std::cos(c.angle);
  const double b = s * std::sin(c.angle);
  const double cx = frame.width * 0.5;
  const double cy = frame.height * 0.5;

  // out = A * (p - centre) + zoom * t + centre
  cv::Mat m(2, 3, CV_64F);
  m.at<double>(0, 0) = a;
  m.at<double>(0, 1) = -b;
  m.at<double>(1, 0) = b;
  m.at<double>(1, 1) = a;
  m.at<double>(0, 2) = zoom * c.dx + cx - (a * cx - b * cy);
  m.at<double>(1, 2) = zoom * c.dy + cy - (b * cx + a * cy);
  return m;
}

}  // namespace stab
