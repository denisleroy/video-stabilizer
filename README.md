# stabilize

A video stabilizer for MP4 files, written in C++20 with OpenCV and FFmpeg. It uses
every CPU core and hardware video encoders where available.

## How it works

The file is decoded once, and frames stream through a pipeline:

```
decode thread ──> motion estimation ──> path smoothing ──> warp (+ GPU upload) ──> encode thread
                  (thread pool, many      (main thread,      (thread pool,          (in order,
                   frame pairs at once)    lookahead window)  many frames at once)    NVENC/VAAPI/QSV/x264)
```

- **Motion:** Shi–Tomasi features and pyramidal Lucas–Kanade flow on a downscaled
  luma plane, then a RANSAC similarity fit (shift, rotation, scale) per frame pair.
  Pairs are independent, so they run in parallel.
- **Smoothing:** Gaussian smoothing of the cumulative camera path. Frame *i* only needs
  the path up to *i + radius*, so a lookahead window replaces a second decode pass.
- **Warp:** each frame is moved by `smoothed − raw` about the centre, plus `--zoom` to
  hide borders. The warp works directly on the Y and UV planes, with no RGB round trip
  and correct chroma siting. Frames are uploaded to the GPU inside the pool.
- **Encode:** hardware first (`<codec>_nvenc` → `_vaapi` → `_qsv` → libx264/libx265).
  Audio is stream-copied, and rotation metadata and colour properties are kept.
- **Inputs** other than 8-bit 4:2:0 (10-bit, 4:2:2, 4:4:4) are converted with
  multithreaded swscale.

## Dependencies (Fedora)

```sh
sudo dnf install cmake ninja-build gcc-c++ opencv-devel ffmpeg-devel   # ffmpeg-devel from RPM Fusion
```

Hardware video (optional):
- Intel: `intel-media-driver` (VAAPI); QSV also needs `libvpl`
- NVIDIA: the proprietary driver (`akmod-nvidia` from RPM Fusion), for NVDEC and NVENC

On Debian/Ubuntu: `libopencv-dev libavformat-dev libavcodec-dev libavutil-dev libswscale-dev`.

## Build

```sh
cmake --preset release && cmake --build --preset release
# or: cmake --preset debug   (ASan + UBSan)
```

## Usage

```sh
scripts/make_shaky.sh shaky.mp4            # synthetic test clip
build/release/stabilize shaky.mp4 out.mp4
```

The first line of output shows the decoder, encoder and thread count chosen.

| Option | Default | |
|---|---|---|
| `--smoothing N` | 30 | smoothing radius in frames; 0 disables |
| `--zoom Z` | 1.1 | zoom used to hide black borders |
| `--encoder E` | auto | `auto`, or an FFmpeg encoder name (`libx264`, `h264_vaapi`, `hevc_nvenc`, …) |
| `--codec C` | h264 | `h264` or `hevc`, for `--encoder auto` |
| `--quality N` | 20 | crf (x264/x265), cq (NVENC), qp (VAAPI), ICQ (QSV); lower is better |
| `--preset P` | | encoder preset (default `medium` for x264/x265, `p5` for NVENC) |
| `--hwdec H` | auto | `auto` (CUDA if present, else software), `none`, `vaapi`, `cuda` |
| `--threads N` | all cores | worker threads for motion estimation and warping |
| `--no-audio` | | drop the audio |
| `--trajectory F` | | write the raw and smoothed path as CSV |

## Performance notes

Measured on an i9-14900HX (32 threads) with Intel UHD (Raptor Lake) graphics, using a
20 s 3840×2160 30 fps H.264 clip (600 frames):

| Configuration | Time | |
|---|---|---|
| First version (two decode passes, BGR, sequential, x264) | 24.5 s | 25 fps |
| Pipeline + libx264 `medium` | 19.2 s | 31 fps |
| Pipeline + h264_vaapi (default here) | 6.0–7.0 s | 85–100 fps |
| FFmpeg CLI: plain decode → h264_vaapi, no processing | 5.0–5.3 s | reference |

- With a hardware encoder, the **encoder is the bottleneck**. The pipeline keeps it
  fed: the encode thread waits on warps for under 0.2 s per run.
- VAAPI *decoding* is not the default because it competes with the VAAPI encoder on
  the same iGPU and measured slower than CPU decoding on this machine. Use
  `--hwdec vaapi` on CPUs with fewer cores.
- Motion estimation and warping run on the CPU. They are not the bottleneck, so moving
  them to the GPU (OpenCV CUDA or OpenCL) would not help. Fedora's OpenCV is also built
  without CUDA modules.
- NVENC is preferred automatically once an NVIDIA driver is installed. With NVDEC
  (`--hwdec auto`), decoding moves off the CPU as well.
- Throughput varies by roughly ±15% between runs on laptops (shared CPU/GPU power and
  thermals), so benchmark with repeated, interleaved runs.

## Layout

```
src/media/   FFmpeg demux/decode with optional hwaccel (VideoReader), encoder
             selection/encode/mux (VideoWriter)
src/stab/    motion estimation, path smoothing, warp matrices, Y/UV-plane warping
src/util/    thread pool, bounded queue
src/main.cpp CLI and the streaming pipeline
```

## Ideas for later

- L1-optimal camera paths (Grundmann et al. 2011) with a crop-window constraint
- Adaptive zoom computed from the largest correction
- Rolling-shutter correction (mesh warps)
- Gyro metadata input (GoPro GPMF, Sony, DJI)
- Keeping 10-bit/HDR sources at 10 bits (currently converted to 8-bit 4:2:0)
