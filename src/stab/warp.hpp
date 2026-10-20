#pragma once

#include "media/av.hpp"

#include <opencv2/core.hpp>

namespace stab {

// Header over the luma plane of an 8-bit frame (no copy).
cv::Mat lumaPlane(const AVFrame& frame);

// Warps an 8-bit 4:2:0 frame (YUV420P, YUVJ420P or NV12) into the
// preallocated `dst` (YUV420P or NV12) using the forward 2x3 luma transform
// `lumaWarp`. Chroma is warped at its own resolution; exposed borders are black.
void warpFrame(const AVFrame& src, AVFrame& dst, const cv::Mat& lumaWarp);

}  // namespace stab
