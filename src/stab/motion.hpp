#pragma once

#include "stab/similarity.hpp"

#include <opencv2/core.hpp>

namespace stab {

// Estimates the camera motion between two consecutive luma planes with sparse
// optical flow and a RANSAC-fitted similarity transform, computed on copies
// downscaled to `analysisWidth`. Returns the motion mapping `prev` onto `curr`
// in full-resolution pixels, or identity when tracking fails.
// Stateless, so pairs of frames can be processed in parallel.
Similarity estimateMotion(const cv::Mat& prev, const cv::Mat& curr, int analysisWidth = 640);

}  // namespace stab
