# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and run

C++20, CMake + Ninja, depends on OpenCV 4 or 5 (5 split modules: `geometry`/`features` replace `calib3d` and part of `imgproc`; handled in `CMakeLists.txt` and `src/stab/motion.cpp`) and FFmpeg (`libavformat libavcodec libavutil libswscale`, found via pkg-config).

```sh
cmake --preset release && cmake --build --preset release   # -> build/release/stabilize
cmake --preset debug   && cmake --build --preset debug     # ASan + UBSan, -> build/debug/stabilize
```

There is no test suite. To exercise changes, generate a synthetic shaky clip (needs the `ffmpeg` CLI) and run on it:

```sh
scripts/make_shaky.sh shaky.mp4
build/release/stabilize shaky.mp4 out.mp4 --trajectory path.csv
```

The first stderr line reports the chosen decoder/encoder/thread count; the last lines report fps and mean raw vs. smoothed camera motion (px/frame), which is the quickest sanity check that stabilization worked. `--encoder libx264` and `--hwdec none` force the software paths. CLI options are documented in README.md and parsed in `src/main.cpp`.

Performance numbers vary ~±15% run to run on laptops; benchmark with repeated, interleaved runs.

## Architecture

All code is in namespace `stab`. `stabilize_core` (static lib: `media/`, `stab/`, `util/`) is linked by the single executable `src/main.cpp`.

**Single-pass streaming pipeline, orchestrated in `run()` in `src/main.cpp`:**

```
decode thread ─(BoundedQueue<FrameRef>, cap 16)─> main thread ─(BoundedQueue<future<FrameRef>>, cap = threads)─> encode thread
                                                    │ submits motion tasks + warp tasks to ThreadPool
```

- The main thread submits `estimateMotion(prev, curr)` for each consecutive frame pair to the pool, keeping a deque of futures. It resolves them in order into a **cumulative camera path** (`std::vector<Similarity>`).
- Frame *i* is warped once the path is known up to *i + radius*: `smoothAt(path, i, radius)` evaluates the Gaussian-smoothed path using only that lookahead window, so no second decode pass is needed. The correction is `smoothed − raw`, turned into a 2×3 matrix by `correctionMatrix` (applied about the frame centre, plus `--zoom`).
- Warp tasks return `future<FrameRef>`s pushed **in order** into the `warped` queue, so the encode thread gets frames in order while warps run in parallel. Back-pressure comes from the bounded queues.
- Errors: any thread reports via `FirstError`, which cancels both queues so the other threads unwind; the exception is rethrown after joins. The `ThreadPool` is declared after the `VideoWriter` on purpose, so pending tasks are destroyed before the writer.

**Key invariants across files:**

- `Similarity` (`stab/similarity.hpp`) stores `dx, dy, angle, logScale` as parameters so paths can be summed/smoothed component-wise. Keep that representation when changing smoothing.
- Frames stay in YUV the whole way, with no RGB round trip. `VideoReader` always delivers system-memory 8-bit 4:2:0 (`YUV420P`, `YUVJ420P`, or `NV12`), converting anything else with swscale and downloading hw frames. `warpFrame` warps Y and chroma planes separately at their own resolution, writing into a frame from `VideoWriter::allocFrame()` (NV12 or YUV420P, whichever the encoder wants). Any new pixel format must be handled in both reader normalization and `warp.cpp`.
- `estimateMotion` is stateless and works on a downscaled luma plane (`lumaPlane()` is a zero-copy `cv::Mat` header over the AVFrame), so pairs can run in parallel.
- `VideoWriter` thread-safety contract: `allocFrame()`/`upload()` may be called from any pool thread (GPU upload happens in the pool for VAAPI); `writeVideo()` from one thread only; `writeAudio()` concurrently with it (muxing is guarded by `muxMutex_`). Audio packets are stream-copied via the `VideoReader::onAudioPacket` callback during decode.
- Encoder auto-selection: `<codec>_nvenc` → `_vaapi` → `_qsv` → libx264/libx265; on macOS (`#ifdef __APPLE__`) `<codec>_videotoolbox` → libx264/libx265, with `--quality` mapped to VideoToolbox constant quality. Hardware decode `auto` means CUDA if present, else software; on macOS always software (VAAPI and VideoToolbox decode are deliberately not defaults: they measured no faster, since the hardware encoder is the bottleneck). Hardware probing is wrapped in `av::QuietLog`.
- Output dimensions are rounded down to even. Rotation (display matrix) and colour properties are copied from the input stream.
- `media/av.hpp` holds the RAII wrappers (`CodecContextPtr`, `PacketPtr`, shared `FrameRef`) and `av::check()` for FFmpeg error codes; use those rather than raw FFmpeg alloc/free.
