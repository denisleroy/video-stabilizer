#!/usr/bin/env bash
# Generates a synthetic shaky 1280x720 clip with audio for testing.
# Usage: scripts/make_shaky.sh [output.mp4]
set -euo pipefail
out=${1:-shaky.mp4}

ffmpeg -y -hide_banner -loglevel error \
  -f lavfi -i "testsrc2=size=1440x900:rate=30:duration=10" \
  -f lavfi -i "sine=frequency=440:duration=10" \
  -vf "crop=1280:720:x='80+40*sin(n*0.7)+25*sin(n*2.3)':y='90+30*sin(n*0.9+1)+20*cos(n*1.9)',rotate='0.02*sin(n*0.5)'" \
  -c:v libx264 -crf 20 -pix_fmt yuv420p -c:a aac -shortest "$out"

echo "wrote $out"
