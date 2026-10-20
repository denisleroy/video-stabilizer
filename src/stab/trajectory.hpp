#pragma once

#include "stab/similarity.hpp"

#include <opencv2/core.hpp>

#include <cstddef>
#include <vector>

namespace stab {

// Gaussian-smoothed value of `path` at index i over +/- radius frames, with the
// path's ends replicated. Only needs path[0 .. i + radius], so it can be
// evaluated while the path is still growing (the final frames once the path
// is complete).
Similarity smoothAt(const std::vector<Similarity>& path, size_t i, int radius);

// smoothAt() for every index of a complete path.
std::vector<Similarity> smoothGaussian(const std::vector<Similarity>& path, int radius);

// 2x3 warp that applies `correction` about the frame centre, then zooms by
// `zoom` about the centre to push empty borders out of view.
cv::Mat correctionMatrix(const Similarity& correction, cv::Size frame, double zoom);

}  // namespace stab
