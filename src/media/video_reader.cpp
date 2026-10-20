#include "media/video_reader.hpp"

#include <cstdint>

namespace stab {

namespace {

// Prefers the hardware format stashed in ctx->opaque, falling back to the
// first software format if the stream turns out to be unsupported by the GPU.
AVPixelFormat getFormat(AVCodecContext* ctx, const AVPixelFormat* fmts) {
  const auto hw = static_cast<AVPixelFormat>(reinterpret_cast<intptr_t>(ctx->opaque));
  for (const AVPixelFormat* p = fmts; *p != AV_PIX_FMT_NONE; ++p) {
    if (*p == hw) return hw;
  }
  for (const AVPixelFormat* p = fmts; *p != AV_PIX_FMT_NONE; ++p) {
    if (!(av_pix_fmt_desc_get(*p)->flags & AV_PIX_FMT_FLAG_HWACCEL)) return *p;
  }
  return AV_PIX_FMT_NONE;
}

bool isWarpable(int format) {
  return format == AV_PIX_FMT_YUV420P || format == AV_PIX_FMT_YUVJ420P ||
         format == AV_PIX_FMT_NV12;
}

}  // namespace

VideoReader::VideoReader(const std::string& path, const std::string& hwdec) {
  av::check(avformat_open_input(&fmt_, path.c_str(), nullptr, nullptr), "open " + path);
  av::check(avformat_find_stream_info(fmt_, nullptr), "find stream info");

  const AVCodec* codec = nullptr;
  videoIdx_ = av_find_best_stream(fmt_, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
  av::check(videoIdx_, "find video stream");
  audioIdx_ = av_find_best_stream(fmt_, AVMEDIA_TYPE_AUDIO, -1, videoIdx_, nullptr, 0);
  if (audioIdx_ < 0) audioIdx_ = -1;

  AVStream* vs = fmt_->streams[videoIdx_];
  frameRate_ = av_guess_frame_rate(fmt_, vs, nullptr);
  if (frameRate_.num <= 0 || frameRate_.den <= 0) frameRate_ = {30, 1};

  dec_.reset(avcodec_alloc_context3(codec));
  if (!dec_) throw std::runtime_error("alloc decoder context");
  av::check(avcodec_parameters_to_context(dec_.get(), vs->codecpar), "copy decoder params");
  dec_->thread_count = 0;  // auto; used by software decoding
  dec_->pkt_timebase = vs->time_base;

  if (hwdec == "auto") {
    // NVDEC is a dedicated engine, so it never slows the encoder. VAAPI is not
    // tried: on an iGPU it competes with the hardware encoder, and measured
    // slower than software decoding on a many-core CPU.
    tryHwDevice(codec, AV_HWDEVICE_TYPE_CUDA);
  } else if (hwdec != "none") {
    const AVHWDeviceType type = av_hwdevice_find_type_by_name(hwdec.c_str());
    if (type == AV_HWDEVICE_TYPE_NONE || !tryHwDevice(codec, type)) {
      throw std::runtime_error("hardware decoder '" + hwdec + "' is not available for " +
                               codec->name);
    }
  }
  av::check(avcodec_open2(dec_.get(), codec, nullptr), "open decoder");

  decoded_ = av::allocFrame();
  pkt_.reset(av_packet_alloc());
  if (!pkt_) throw std::bad_alloc();
}

VideoReader::~VideoReader() {
  sws_freeContext(sws_);
  avformat_close_input(&fmt_);
}

bool VideoReader::tryHwDevice(const AVCodec* codec, AVHWDeviceType type) {
  for (int i = 0;; ++i) {
    const AVCodecHWConfig* cfg = avcodec_get_hw_config(codec, i);
    if (!cfg) return false;
    if (!(cfg->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) || cfg->device_type != type) {
      continue;
    }
    AVBufferRef* device = nullptr;
    {
      av::QuietLog quiet;
      if (av_hwdevice_ctx_create(&device, type, nullptr, nullptr, 0) < 0) return false;
    }
    dec_->hw_device_ctx = device;
    dec_->opaque = reinterpret_cast<void*>(static_cast<intptr_t>(cfg->pix_fmt));
    dec_->get_format = getFormat;
    hwPixFmt_ = cfg->pix_fmt;
    hwName_ = av_hwdevice_get_type_name(type);
    return true;
  }
}

std::string VideoReader::decoderName() const {
  return std::string(dec_->codec->name) + " (" + (hwName_.empty() ? "software" : hwName_) + ")";
}

av::FrameRef VideoReader::normalize(av::FrameRef frame) {
  if (isWarpable(frame->format)) return frame;

  // e.g. 10-bit or 4:2:2 sources: convert to 8-bit 4:2:0 on all cores.
  const auto srcFmt = static_cast<AVPixelFormat>(frame->format);
  if (!sws_) {
    sws_ = sws_alloc_context();
    if (!sws_) throw std::bad_alloc();
    av_opt_set_int(sws_, "threads", 0, 0);
    av_opt_set_int(sws_, "sws_flags", SWS_BILINEAR, 0);
  }
  av::FrameRef out = av::allocFrame();
  av::check(av_frame_copy_props(out.get(), frame.get()), "copy frame props");
  out->format = AV_PIX_FMT_YUV420P;
  out->width = frame->width;
  out->height = frame->height;
  av::check(sws_scale_frame(sws_, out.get(), frame.get()),
            std::string("convert from ") + av_get_pix_fmt_name(srcFmt));
  return out;
}

bool VideoReader::read(av::FrameRef& out) {
  for (;;) {
    int r = avcodec_receive_frame(dec_.get(), decoded_.get());
    if (r == 0) {
      av::FrameRef frame = av::allocFrame();
      if (decoded_->format == hwPixFmt_) {
        av::check(av_hwframe_transfer_data(frame.get(), decoded_.get(), 0), "download GPU frame");
        av::check(av_frame_copy_props(frame.get(), decoded_.get()), "copy frame props");
        av_frame_unref(decoded_.get());
      } else {
        av_frame_move_ref(frame.get(), decoded_.get());
      }
      out = normalize(std::move(frame));

      int64_t pts = out->best_effort_timestamp;
      if (pts == AV_NOPTS_VALUE) {
        pts = av_rescale_q(frameCount_, av_inv_q(frameRate_), videoStream()->time_base);
      }
      out->pts = pts;
      ++frameCount_;
      return true;
    }
    if (r == AVERROR_EOF) return false;
    if (r != AVERROR(EAGAIN)) av::check(r, "decode video");

    // Decoder needs more input.
    r = av_read_frame(fmt_, pkt_.get());
    if (r == AVERROR_EOF) {
      av::check(avcodec_send_packet(dec_.get(), nullptr), "flush decoder");
      continue;
    }
    av::check(r, "read packet");

    if (pkt_->stream_index == videoIdx_) {
      r = avcodec_send_packet(dec_.get(), pkt_.get());
      if (r < 0 && r != AVERROR_INVALIDDATA) av::check(r, "send packet");
    } else if (pkt_->stream_index == audioIdx_ && onAudioPacket) {
      onAudioPacket(pkt_.get());
    }
    av_packet_unref(pkt_.get());
  }
}

}  // namespace stab
