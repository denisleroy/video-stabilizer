#include "media/video_reader.hpp"
#include "media/video_writer.hpp"
#include "stab/motion.hpp"
#include "stab/trajectory.hpp"
#include "stab/warp.hpp"
#include "util/bounded_queue.hpp"
#include "util/thread_pool.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <exception>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
  std::string input;
  std::string output;
  int smoothingRadius = 30;
  double zoom = 1.1;
  bool keepAudio = true;
  std::string hwdec = "auto";
  unsigned threads = 0;  // 0 = all cores
  std::string trajectoryCsv;
  stab::EncoderOptions encoder;
};

void printUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " [options] <input.mp4> <output.mp4>\n"
            << "\n"
            << "Stabilization:\n"
            << "  --smoothing N     smoothing radius in frames (default 30, 0 = off)\n"
            << "  --zoom Z          zoom factor to hide borders (default 1.1)\n"
            << "\n"
            << "Encoding:\n"
            << "  --encoder E       auto (default: nvenc > vaapi > qsv > x264/x265) or an\n"
            << "                    FFmpeg encoder name, e.g. libx264, h264_vaapi\n"
            << "  --codec C         h264 (default) or hevc, used by --encoder auto\n"
            << "  --quality N       crf/cq/qp, lower is better (default 20)\n"
            << "  --preset P        encoder preset (default: medium for x264/x265, p5 for nvenc)\n"
            << "  --no-audio        drop the audio track\n"
            << "\n"
            << "Resources:\n"
            << "  --hwdec H         auto (default: cuda, else software), none, vaapi, cuda\n"
            << "  --threads N       worker threads for analysis and warping (default: all cores)\n"
            << "\n"
            << "  --trajectory F    write raw and smoothed camera path to CSV file F\n"
            << "  -h, --help        show this help\n";
}

bool parseArgs(int argc, char** argv, Options& opts) {
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
      return argv[++i];
    };
    if (arg == "-h" || arg == "--help") return false;
    if (arg == "--smoothing") opts.smoothingRadius = std::stoi(value());
    else if (arg == "--zoom") opts.zoom = std::stod(value());
    else if (arg == "--encoder") opts.encoder.encoder = value();
    else if (arg == "--codec") opts.encoder.codec = value();
    else if (arg == "--quality") opts.encoder.quality = std::stoi(value());
    else if (arg == "--preset") opts.encoder.preset = value();
    else if (arg == "--no-audio") opts.keepAudio = false;
    else if (arg == "--hwdec") opts.hwdec = value();
    else if (arg == "--threads") opts.threads = static_cast<unsigned>(std::stoul(value()));
    else if (arg == "--trajectory") opts.trajectoryCsv = value();
    else if (arg.starts_with("-")) throw std::runtime_error("unknown option " + arg);
    else positional.push_back(arg);
  }
  if (positional.size() != 2) return false;
  opts.input = positional[0];
  opts.output = positional[1];
  if (opts.zoom < 1.0) throw std::runtime_error("--zoom must be >= 1");
  if (opts.smoothingRadius < 0) throw std::runtime_error("--smoothing must be >= 0");
  return true;
}

double meanStep(const std::vector<stab::Similarity>& path) {
  if (path.size() < 2) return 0;
  double sum = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    sum += std::hypot(path[i].dx - path[i - 1].dx, path[i].dy - path[i - 1].dy);
  }
  return sum / static_cast<double>(path.size() - 1);
}

void writeTrajectory(const std::string& file, const std::vector<stab::Similarity>& raw,
                     const std::vector<stab::Similarity>& smooth) {
  std::ofstream out(file);
  out << "frame,x,y,angle,log_scale,smooth_x,smooth_y,smooth_angle,smooth_log_scale\n";
  for (size_t i = 0; i < raw.size(); ++i) {
    out << i << ',' << raw[i].dx << ',' << raw[i].dy << ',' << raw[i].angle << ','
        << raw[i].logScale << ',' << smooth[i].dx << ',' << smooth[i].dy << ','
        << smooth[i].angle << ',' << smooth[i].logScale << '\n';
  }
}

// Records the first error from any pipeline thread and runs `onError` once.
class FirstError {
 public:
  template <class F>
  explicit FirstError(F onError) : onError_(std::move(onError)) {}

  void set(std::exception_ptr e) {
    {
      std::lock_guard lock(mutex_);
      if (error_) return;
      error_ = e;
    }
    onError_();
  }
  bool has() {
    std::lock_guard lock(mutex_);
    return error_ != nullptr;
  }
  void rethrow() {
    std::lock_guard lock(mutex_);
    if (error_) std::rethrow_exception(error_);
  }

 private:
  std::mutex mutex_;
  std::exception_ptr error_;
  std::function<void()> onError_;
};

// Single-pass pipeline:
//
//   decode thread ──> main: schedule motion (pool) ──> main: smooth + schedule warp (pool)
//                                                      ──> encode thread (in order)
//
// Frame i is warped once the path is known up to i + radius. Motion results are
// awaited `threads` frames behind the newest decoded frame, so motion estimation
// for that many frame pairs runs concurrently; warps run concurrently with it.
int run(const Options& opts) {
  const unsigned threads = opts.threads > 0 ? opts.threads : std::max(1u, std::thread::hardware_concurrency());
  const auto start = std::chrono::steady_clock::now();

  stab::VideoReader reader(opts.input, opts.hwdec);
  // 4:2:0 encoders need even dimensions.
  const int outW = reader.width() & ~1;
  const int outH = reader.height() & ~1;
  const AVStream* audio = opts.keepAudio ? reader.audioStream() : nullptr;
  stab::VideoWriter writer(opts.output, outW, outH, reader.frameRate(), reader.videoStream(), audio,
                           opts.encoder);
  if (audio) reader.onAudioPacket = [&](AVPacket* pkt) { writer.writeAudio(pkt); };
  std::fprintf(stderr, "decoder: %s, encoder: %s, %u worker threads\n", reader.decoderName().c_str(),
               writer.encoderName().c_str(), threads);

  stab::BoundedQueue<stab::av::FrameRef> decoded(16);
  stab::BoundedQueue<std::future<stab::av::FrameRef>> warped(threads);
  FirstError error([&] {
    decoded.cancel();
    warped.cancel();
  });
  // Declared after the writer, so pending tasks are dropped before it is destroyed.
  stab::ThreadPool pool(threads);

  std::thread decodeThread([&] {
    try {
      stab::av::FrameRef frame;
      while (reader.read(frame)) {
        if (!decoded.push(std::move(frame))) return;
      }
      decoded.close();
    } catch (...) {
      error.set(std::current_exception());
    }
  });

  std::atomic<size_t> encoded = 0;
  std::thread encodeThread([&] {
    try {
      while (auto next = warped.pop()) {
        writer.writeVideo(next->get().get());
        if (++encoded % 50 == 0) std::fprintf(stderr, "\rencoded %zu frames", encoded.load());
      }
    } catch (...) {
      error.set(std::current_exception());
    }
  });

  const int radius = opts.smoothingRadius;
  std::vector<stab::Similarity> path;                 // cumulative camera path, frames [0, path.size())
  std::deque<std::future<stab::Similarity>> motions;  // frames [path.size(), received)
  std::deque<stab::av::FrameRef> frames;              // frames [nextWarp, received)
  size_t received = 0;
  size_t nextWarp = 0;
  stab::av::FrameRef prev;

  auto resolvePath = [&](size_t count) {
    while (path.size() < count) {
      const stab::Similarity m = motions.front().get();
      motions.pop_front();
      path.push_back(path.empty() ? m : path.back() + m);
    }
  };
  auto scheduleWarp = [&] {
    const size_t i = nextWarp++;
    stab::av::FrameRef src = std::move(frames.front());
    frames.pop_front();
    const stab::Similarity correction = stab::smoothAt(path, i, radius) - path[i];
    const cv::Mat m = stab::correctionMatrix(correction, {src->width, src->height}, opts.zoom);
    return warped.push(pool.submit([src, m, &writer] {
      stab::av::FrameRef out = writer.allocFrame();
      stab::warpFrame(*src, *out, m);
      stab::av::check(av_frame_copy_props(out.get(), src.get()), "copy frame props");
      out->pict_type = AV_PICTURE_TYPE_NONE;  // let the encoder choose keyframes
      return writer.upload(std::move(out));
    }));
  };

  try {
    while (auto next = decoded.pop()) {
      stab::av::FrameRef frame = std::move(*next);
      if (prev) {
        motions.push_back(pool.submit([prev, frame] {
          return stab::estimateMotion(stab::lumaPlane(*prev), stab::lumaPlane(*frame));
        }));
      } else {
        std::promise<stab::Similarity> identity;
        identity.set_value({});
        motions.push_back(identity.get_future());
      }
      prev = frame;
      frames.push_back(std::move(frame));
      ++received;

      while (received > nextWarp + radius + threads) {
        resolvePath(nextWarp + radius + 1);
        if (!scheduleWarp()) break;
      }
    }
    if (!error.has()) {
      resolvePath(received);
      while (nextWarp < received && scheduleWarp()) {
      }
    }
    warped.close();
  } catch (...) {
    error.set(std::current_exception());
  }

  decodeThread.join();
  encodeThread.join();
  error.rethrow();
  writer.finish();
  if (path.empty()) throw std::runtime_error("no video frames decoded");

  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  const auto smooth = stab::smoothGaussian(path, radius);
  std::fprintf(stderr, "\rencoded %zu frames in %.2f s (%.1f fps)\n", encoded.load(), seconds,
               encoded.load() / seconds);
  std::fprintf(stderr, "mean camera motion: %.2f px/frame raw, %.2f px/frame smoothed\n",
               meanStep(path), meanStep(smooth));
  if (!opts.trajectoryCsv.empty()) writeTrajectory(opts.trajectoryCsv, path, smooth);
  std::fprintf(stderr, "wrote %s\n", opts.output.c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Options opts;
    if (!parseArgs(argc, argv, opts)) {
      printUsage(argv[0]);
      return 2;
    }
    av_log_set_level(AV_LOG_ERROR);
    return run(opts);
  } catch (const std::exception& e) {
    std::cerr << "\nerror: " << e.what() << '\n';
    return 1;
  }
}
