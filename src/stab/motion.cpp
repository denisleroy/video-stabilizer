#include "stab/motion.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <vector>

namespace stab {

namespace {

constexpr int kMaxFeatures = 400;
constexpr size_t kMinTracked = 12;
constexpr double kRansacThresholdPx = 3.0;

cv::Mat downscale(const cv::Mat& luma, double scale) {
  if (scale >= 1.0) return luma;
  cv::Mat small;
  cv::resize(luma, small, {}, scale, scale, cv::INTER_AREA);
  return small;
}

}  // namespace

Similarity estimateMotion(const cv::Mat& prev, const cv::Mat& curr, int analysisWidth) {
  Similarity motion;
  if (prev.size() != curr.size()) return motion;

  const double scale = prev.cols > analysisWidth ? static_cast<double>(analysisWidth) / prev.cols : 1.0;
  const cv::Mat prevSmall = downscale(prev, scale);
  const cv::Mat currSmall = downscale(curr, scale);

  std::vector<cv::Point2f> prevPts;
  cv::goodFeaturesToTrack(prevSmall, prevPts, kMaxFeatures, 0.01, 8);
  if (prevPts.size() < kMinTracked) return motion;

  std::vector<cv::Point2f> currPts;
  std::vector<uchar> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(prevSmall, currSmall, prevPts, currPts, status, err);

  // Express points relative to the frame centre so rotation/scale are
  // estimated about the centre, which is where corrections are applied.
  const cv::Point2f centre(prevSmall.cols * 0.5f, prevSmall.rows * 0.5f);
  std::vector<cv::Point2f> p0, p1;
  for (size_t i = 0; i < status.size(); ++i) {
    if (!status[i]) continue;
    p0.push_back(prevPts[i] - centre);
    p1.push_back(currPts[i] - centre);
  }
  if (p0.size() < kMinTracked) return motion;

  const cv::Mat m = cv::estimateAffinePartial2D(p0, p1, cv::noArray(), cv::RANSAC, kRansacThresholdPx);
  if (m.empty()) return motion;

  const double a = m.at<double>(0, 0);
  const double b = m.at<double>(1, 0);
  motion.dx = m.at<double>(0, 2) / scale;
  motion.dy = m.at<double>(1, 2) / scale;
  motion.angle = std::atan2(b, a);
  motion.logScale = std::log(std::hypot(a, b));
  return motion;
}

}  // namespace stab
