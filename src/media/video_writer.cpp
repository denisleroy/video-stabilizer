#include "media/video_writer.hpp"

#include <cstring>
#include <string>
#include <vector>

namespace stab {

namespace {

bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool supportsPixFmt(const AVCodecContext* ctx, const AVCodec* codec, AVPixelFormat fmt) {
  const void* configs = nullptr;
  int count = 0;
  if (avcodec_get_supported_config(ctx, codec, AV_CODEC_CONFIG_PIX_FORMAT, 0, &configs, &count) < 0) {
    return false;
  }
  if (!configs) return true;  // unrestricted
  const auto* fmts = static_cast<const AVPixelFormat*>(configs);
  for (int i = 0; i < count; ++i) {
    if (fmts[i] == fmt) return true;
  }
  return false;
}

// Attaches an NV12 VAAPI surface pool so frames can be uploaded to the GPU.
bool attachVaapiFrames(AVCodecContext* enc) {
  AVBufferRef* device = nullptr;
  if (av_hwdevice_ctx_create(&device, AV_HWDEVICE_TYPE_VAAPI, nullptr, nullptr, 0) < 0) return false;
  AVBufferRef* frames = av_hwframe_ctx_alloc(device);
  av_buffer_unref(&device);
  if (!frames) return false;
  auto* fc = reinterpret_cast<AVHWFramesContext*>(frames->data);
  fc->format = AV_PIX_FMT_VAAPI;
  fc->sw_format = AV_PIX_FMT_NV12;
  fc->width = enc->width;
  fc->height = enc->height;
  fc->initial_pool_size = 0;  // grow on demand: many frames are uploaded ahead of the encoder
  if (av_hwframe_ctx_init(frames) < 0) {
    av_buffer_unref(&frames);
    return false;
  }
  enc->hw_frames_ctx = frames;
  return true;
}

}  // namespace

VideoWriter::VideoWriter(const std::string& path, int width, int height, AVRational frameRate,
                         const AVStream* videoSrc, const AVStream* audioSrc,
                         const EncoderOptions& opts) {
  av::check(avformat_alloc_output_context2(&fmt_, nullptr, nullptr, path.c_str()),
            "create output for " + path);

  std::vector<std::string> candidates;
  if (opts.encoder == "auto") {
    const bool hevc = opts.codec == "hevc";
    if (!hevc && opts.codec != "h264") throw std::runtime_error("unknown codec " + opts.codec);
    for (const char* hw : {"_nvenc", "_vaapi", "_qsv"}) candidates.push_back(opts.codec + hw);
    candidates.push_back(hevc ? "libx265" : "libx264");
  } else {
    candidates.push_back(opts.encoder);
  }

  for (const auto& name : candidates) {
    const AVCodec* codec = avcodec_find_encoder_by_name(name.c_str());
    if (!codec) continue;
    // Only silence errors while probing; an explicitly chosen encoder should explain itself.
    std::unique_ptr<av::QuietLog> quiet;
    if (candidates.size() > 1) quiet = std::make_unique<av::QuietLog>();
    if (openEncoder(codec, width, height, frameRate, videoSrc, opts)) break;
  }
  if (!enc_) {
    std::string tried;
    for (const auto& name : candidates) tried += (tried.empty() ? "" : ", ") + name;
    throw std::runtime_error("could not open encoder (tried " + tried + ")");
  }

  videoOut_ = avformat_new_stream(fmt_, nullptr);
  if (!videoOut_) throw std::runtime_error("create video stream");
  av::check(avcodec_parameters_from_context(videoOut_->codecpar, enc_.get()), "copy encoder params");
  videoOut_->time_base = enc_->time_base;
  videoOut_->avg_frame_rate = frameRate;

  // Preserve rotation metadata (phone footage) so the output displays the same way.
  const AVCodecParameters* srcPar = videoSrc->codecpar;
  for (int i = 0; i < srcPar->nb_coded_side_data; ++i) {
    const AVPacketSideData& sd = srcPar->coded_side_data[i];
    if (sd.type != AV_PKT_DATA_DISPLAYMATRIX) continue;
    AVPacketSideData* dst = av_packet_side_data_new(&videoOut_->codecpar->coded_side_data,
                                                    &videoOut_->codecpar->nb_coded_side_data,
                                                    sd.type, sd.size, 0);
    if (dst) memcpy(dst->data, sd.data, sd.size);
  }

  if (audioSrc) {
    audioOut_ = avformat_new_stream(fmt_, nullptr);
    if (!audioOut_) throw std::runtime_error("create audio stream");
    av::check(avcodec_parameters_copy(audioOut_->codecpar, audioSrc->codecpar), "copy audio params");
    audioOut_->codecpar->codec_tag = 0;
    audioOut_->time_base = audioSrc->time_base;
    audioSrcTimeBase_ = audioSrc->time_base;
  }

  if (!(fmt_->oformat->flags & AVFMT_NOFILE)) {
    av::check(avio_open(&fmt_->pb, path.c_str(), AVIO_FLAG_WRITE), "open " + path);
  }
  // Audio arrives well ahead of encoded video; always wait for both so the
  // output is properly interleaved.
  fmt_->max_interleave_delta = 0;
  AVDictionary* muxOpts = nullptr;
  av_dict_set(&muxOpts, "movflags", "+faststart", 0);
  const int r = avformat_write_header(fmt_, &muxOpts);
  av_dict_free(&muxOpts);
  av::check(r, "write header");

  pkt_.reset(av_packet_alloc());
  if (!pkt_) throw std::bad_alloc();
}

VideoWriter::~VideoWriter() {
  if (fmt_ && !(fmt_->oformat->flags & AVFMT_NOFILE)) avio_closep(&fmt_->pb);
  avformat_free_context(fmt_);
}

bool VideoWriter::openEncoder(const AVCodec* codec, int width, int height, AVRational frameRate,
                              const AVStream* videoSrc, const EncoderOptions& opts) {
  av::CodecContextPtr enc(avcodec_alloc_context3(codec));
  if (!enc) return false;
  const AVCodecParameters* src = videoSrc->codecpar;
  enc->width = width;
  enc->height = height;
  enc->time_base = videoSrc->time_base;
  enc->framerate = frameRate;
  enc->sample_aspect_ratio = src->sample_aspect_ratio;
  enc->color_range = src->color_range;
  enc->color_primaries = src->color_primaries;
  enc->color_trc = src->color_trc;
  enc->colorspace = src->color_space;
  enc->chroma_sample_location = src->chroma_location;
  enc->thread_count = 0;
  if (fmt_->oformat->flags & AVFMT_GLOBALHEADER) enc->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  const std::string name = codec->name;
  const std::string quality = std::to_string(opts.quality);
  AVDictionary* o = nullptr;
  AVPixelFormat sw = AV_PIX_FMT_NONE;

  if (endsWith(name, "_vaapi")) {
    if (!attachVaapiFrames(enc.get())) return false;
    enc->pix_fmt = AV_PIX_FMT_VAAPI;
    sw = AV_PIX_FMT_NV12;
    av_dict_set(&o, "rc_mode", "CQP", 0);
    av_dict_set(&o, "qp", quality.c_str(), 0);
  } else {
    if (supportsPixFmt(enc.get(), codec, AV_PIX_FMT_NV12)) sw = AV_PIX_FMT_NV12;
    else if (supportsPixFmt(enc.get(), codec, AV_PIX_FMT_YUV420P)) sw = AV_PIX_FMT_YUV420P;
    else return false;
    enc->pix_fmt = sw;

    if (endsWith(name, "_nvenc")) {
      av_dict_set(&o, "preset", opts.preset.empty() ? "p5" : opts.preset.c_str(), 0);
      av_dict_set(&o, "tune", "hq", 0);
      av_dict_set(&o, "rc", "vbr", 0);
      av_dict_set(&o, "cq", quality.c_str(), 0);
      enc->bit_rate = 0;
    } else if (endsWith(name, "_qsv")) {
      enc->global_quality = opts.quality;  // ICQ
      if (!opts.preset.empty()) av_dict_set(&o, "preset", opts.preset.c_str(), 0);
    } else {
      av_dict_set(&o, "crf", quality.c_str(), 0);
      av_dict_set(&o, "preset", opts.preset.empty() ? "medium" : opts.preset.c_str(), 0);
    }
  }

  const int r = avcodec_open2(enc.get(), codec, &o);
  av_dict_free(&o);
  if (r < 0) return false;

  enc_ = std::move(enc);
  swFormat_ = sw;
  encoderName_ = name;
  return true;
}

av::FrameRef VideoWriter::allocFrame() const {
  av::FrameRef f = av::allocFrame();
  f->format = swFormat_;
  f->width = enc_->width;
  f->height = enc_->height;
  av::check(av_frame_get_buffer(f.get(), 0), "alloc frame buffer");
  return f;
}

av::FrameRef VideoWriter::upload(av::FrameRef frame) const {
  if (enc_->pix_fmt != AV_PIX_FMT_VAAPI) return frame;
  av::FrameRef gpu = av::allocFrame();
  av::check(av_hwframe_get_buffer(enc_->hw_frames_ctx, gpu.get(), 0), "get GPU surface");
  av::check(av_hwframe_transfer_data(gpu.get(), frame.get(), 0), "upload frame to GPU");
  av::check(av_frame_copy_props(gpu.get(), frame.get()), "copy frame props");
  return gpu;
}

void VideoWriter::writeVideo(AVFrame* frame) { encode(frame); }

void VideoWriter::writeAudio(AVPacket* pkt) {
  if (!audioOut_) return;
  av_packet_rescale_ts(pkt, audioSrcTimeBase_, audioOut_->time_base);
  pkt->stream_index = audioOut_->index;
  pkt->pos = -1;
  writePacket(pkt);
}

void VideoWriter::encode(AVFrame* frame) {
  av::check(avcodec_send_frame(enc_.get(), frame), "send frame to encoder");
  for (;;) {
    const int r = avcodec_receive_packet(enc_.get(), pkt_.get());
    if (r == AVERROR(EAGAIN) || r == AVERROR_EOF) return;
    av::check(r, "encode video");
    av_packet_rescale_ts(pkt_.get(), enc_->time_base, videoOut_->time_base);
    pkt_->stream_index = videoOut_->index;
    writePacket(pkt_.get());
  }
}

void VideoWriter::writePacket(AVPacket* pkt) {
  std::lock_guard lock(muxMutex_);
  av::check(av_interleaved_write_frame(fmt_, pkt), "write packet");
}

void VideoWriter::finish() {
  if (finished_) return;
  finished_ = true;
  encode(nullptr);
  std::lock_guard lock(muxMutex_);
  av::check(av_write_trailer(fmt_), "write trailer");
}

}  // namespace stab
