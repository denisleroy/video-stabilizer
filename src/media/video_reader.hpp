#pragma once

#include "media/av.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace stab {

// Demuxes a media file and decodes its best video stream, optionally on a GPU.
// Frames are delivered in system memory as 8-bit 4:2:0 (YUV420P, YUVJ420P or
// NV12); other formats are converted. Packets of the best audio stream can be
// forwarded untouched via onAudioPacket.
class VideoReader {
 public:
  // `hwdec`: "auto" (CUDA if available, else software), "none", or an FFmpeg
  // hwdevice type name such as "vaapi" or "cuda".
  VideoReader(const std::string& path, const std::string& hwdec);
  ~VideoReader();
  VideoReader(const VideoReader&) = delete;
  VideoReader& operator=(const VideoReader&) = delete;

  int width() const { return dec_->width; }
  int height() const { return dec_->height; }
  AVRational frameRate() const { return frameRate_; }
  const AVStream* videoStream() const { return fmt_->streams[videoIdx_]; }
  const AVStream* audioStream() const { return audioIdx_ >= 0 ? fmt_->streams[audioIdx_] : nullptr; }
  // e.g. "h264 (vaapi)" or "hevc (software)".
  std::string decoderName() const;

  // Decodes the next video frame; its pts is in videoStream()->time_base.
  // Returns false at end of stream.
  bool read(av::FrameRef& out);

  // Called with each audio packet encountered while reading. The packet is
  // unreferenced after the callback returns, so it may be consumed by it.
  std::function<void(AVPacket*)> onAudioPacket;

 private:
  bool tryHwDevice(const AVCodec* codec, AVHWDeviceType type);
  av::FrameRef normalize(av::FrameRef frame);

  AVFormatContext* fmt_ = nullptr;
  av::CodecContextPtr dec_;
  av::FrameRef decoded_;
  av::PacketPtr pkt_;
  SwsContext* sws_ = nullptr;
  int videoIdx_ = -1;
  int audioIdx_ = -1;
  AVPixelFormat hwPixFmt_ = AV_PIX_FMT_NONE;
  std::string hwName_;
  AVRational frameRate_{0, 1};
  int64_t frameCount_ = 0;
};

}  // namespace stab
