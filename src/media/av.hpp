#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <memory>
#include <stdexcept>
#include <string>

// Thin RAII and error helpers around the FFmpeg C API.
namespace stab::av {

struct CodecContextDeleter {
  void operator()(AVCodecContext* c) const { avcodec_free_context(&c); }
};
struct PacketDeleter {
  void operator()(AVPacket* p) const { av_packet_free(&p); }
};

using CodecContextPtr = std::unique_ptr<AVCodecContext, CodecContextDeleter>;
using PacketPtr = std::unique_ptr<AVPacket, PacketDeleter>;

// Frames are shared between pipeline stages, so they are reference counted.
using FrameRef = std::shared_ptr<AVFrame>;

inline FrameRef allocFrame() {
  AVFrame* f = av_frame_alloc();
  if (!f) throw std::bad_alloc();
  return FrameRef(f, [](AVFrame* p) { av_frame_free(&p); });
}

inline std::string errorString(int err) {
  char buf[AV_ERROR_MAX_STRING_SIZE]{};
  av_strerror(err, buf, sizeof buf);
  return buf;
}

inline void check(int err, const std::string& what) {
  if (err < 0) throw std::runtime_error(what + ": " + errorString(err));
}

// Silences libav logging for the lifetime of the object (used while probing
// hardware that may legitimately be missing).
class QuietLog {
 public:
  QuietLog() : level_(av_log_get_level()) { av_log_set_level(AV_LOG_QUIET); }
  ~QuietLog() { av_log_set_level(level_); }
  QuietLog(const QuietLog&) = delete;
  QuietLog& operator=(const QuietLog&) = delete;

 private:
  int level_;
};

}  // namespace stab::av
