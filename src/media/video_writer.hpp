#pragma once

#include "media/av.hpp"

#include <mutex>
#include <string>

namespace stab {

struct EncoderOptions {
  // "auto" tries <codec>_nvenc, <codec>_vaapi, <codec>_qsv, then libx264/libx265.
  // Anything else is used as an FFmpeg encoder name.
  std::string encoder = "auto";
  std::string codec = "h264";  // h264 | hevc
  int quality = 20;            // crf / cq / qp / ICQ, lower is better
  std::string preset;          // encoder-specific; empty = sensible default
};

// Encodes 8-bit 4:2:0 frames (on a GPU when available) and muxes them, plus
// an optional stream-copied audio track, into a container chosen from the
// output file extension.
class VideoWriter {
 public:
  // `videoSrc` supplies the time base, colour properties and display matrix of
  // the input; `audioSrc` (may be null) is stream-copied into the output.
  VideoWriter(const std::string& path, int width, int height, AVRational frameRate,
              const AVStream* videoSrc, const AVStream* audioSrc, const EncoderOptions& opts);
  ~VideoWriter();
  VideoWriter(const VideoWriter&) = delete;
  VideoWriter& operator=(const VideoWriter&) = delete;

  const std::string& encoderName() const { return encoderName_; }

  // A blank system-memory frame in the layout the encoder takes (NV12 or
  // YUV420P). Safe to call from any thread.
  av::FrameRef allocFrame() const;

  // Moves a frame from allocFrame() to wherever the encoder reads it from
  // (GPU memory for VAAPI; a no-op otherwise). Safe to call from any thread,
  // so uploads can run in parallel ahead of the encoder.
  av::FrameRef upload(av::FrameRef frame) const;

  // Encodes a frame from upload(); `frame->pts` is in videoSrc->time_base.
  // Call from one thread only.
  void writeVideo(AVFrame* frame);
  // `pkt` timestamps are in audioSrc->time_base. Takes ownership of the
  // packet's data. Safe to call concurrently with writeVideo().
  void writeAudio(AVPacket* pkt);
  // Flushes the encoder and writes the trailer.
  void finish();

 private:
  bool openEncoder(const AVCodec* codec, int width, int height, AVRational frameRate,
                   const AVStream* videoSrc, const EncoderOptions& opts);
  void encode(AVFrame* frame);
  void writePacket(AVPacket* pkt);

  AVFormatContext* fmt_ = nullptr;
  av::CodecContextPtr enc_;
  av::PacketPtr pkt_;
  AVPixelFormat swFormat_ = AV_PIX_FMT_NONE;
  std::string encoderName_;
  AVStream* videoOut_ = nullptr;
  AVStream* audioOut_ = nullptr;
  AVRational audioSrcTimeBase_{0, 1};
  std::mutex muxMutex_;
  bool finished_ = false;
};

}  // namespace stab
