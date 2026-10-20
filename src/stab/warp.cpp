#include "stab/warp.hpp"

#include <opencv2/imgproc.hpp>

#include <vector>

namespace stab {

namespace {

cv::Mat plane(const AVFrame& f, int index, int rows, int cols, int type) {
  return cv::Mat(rows, cols, type, f.data[index], static_cast<size_t>(f.linesize[index]));
}

void warp(const cv::Mat& src, cv::Mat& dst, const cv::Mat& m, const cv::Scalar& border) {
  cv::warpAffine(src, dst, m, dst.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, border);
}

}  // namespace

cv::Mat lumaPlane(const AVFrame& frame) { return plane(frame, 0, frame.height, frame.width, CV_8UC1); }

void warpFrame(const AVFrame& src, AVFrame& dst, const cv::Mat& lumaWarp) {
  const bool fullRange = src.color_range == AVCOL_RANGE_JPEG || src.format == AV_PIX_FMT_YUVJ420P;
  const cv::Scalar black(fullRange ? 0 : 16);
  const cv::Scalar neutral(128, 128);

  cv::Mat dstY = lumaPlane(dst);
  warp(lumaPlane(src), dstY, lumaWarp, black);

  // Same transform in chroma coordinates. With centre-sited 4:2:0 chroma,
  // luma = 2 * chroma + o (o = 0.5), so t' = (A o + t - o) / 2.
  cv::Mat m = lumaWarp.clone();
  const double o = 0.5;
  for (int r = 0; r < 2; ++r) {
    const double ao = (lumaWarp.at<double>(r, 0) + lumaWarp.at<double>(r, 1)) * o;
    m.at<double>(r, 2) = (ao + lumaWarp.at<double>(r, 2) - o) / 2;
  }

  const int srcCw = (src.width + 1) / 2, srcCh = (src.height + 1) / 2;
  const int dstCw = (dst.width + 1) / 2, dstCh = (dst.height + 1) / 2;
  const bool srcNv12 = src.format == AV_PIX_FMT_NV12;
  const bool dstNv12 = dst.format == AV_PIX_FMT_NV12;

  if (srcNv12 && dstNv12) {
    cv::Mat dstUV = plane(dst, 1, dstCh, dstCw, CV_8UC2);
    warp(plane(src, 1, srcCh, srcCw, CV_8UC2), dstUV, m, neutral);
  } else if (!srcNv12 && !dstNv12) {
    for (int p = 1; p <= 2; ++p) {
      cv::Mat dstC = plane(dst, p, dstCh, dstCw, CV_8UC1);
      warp(plane(src, p, srcCh, srcCw, CV_8UC1), dstC, m, neutral);
    }
  } else if (dstNv12) {  // planar -> interleaved
    cv::Mat u(dstCh, dstCw, CV_8UC1), v(dstCh, dstCw, CV_8UC1);
    warp(plane(src, 1, srcCh, srcCw, CV_8UC1), u, m, neutral);
    warp(plane(src, 2, srcCh, srcCw, CV_8UC1), v, m, neutral);
    cv::Mat dstUV = plane(dst, 1, dstCh, dstCw, CV_8UC2);
    cv::merge(std::vector<cv::Mat>{u, v}, dstUV);
  } else {  // interleaved -> planar
    cv::Mat uv(dstCh, dstCw, CV_8UC2);
    warp(plane(src, 1, srcCh, srcCw, CV_8UC2), uv, m, neutral);
    cv::Mat out[] = {plane(dst, 1, dstCh, dstCw, CV_8UC1), plane(dst, 2, dstCh, dstCw, CV_8UC1)};
    cv::split(uv, out);
  }
}

}  // namespace stab
