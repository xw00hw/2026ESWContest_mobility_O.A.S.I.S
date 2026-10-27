#!/usr/bin/env bash
# Download the MediaPipe Pose Landmarker bundle.
#
# The model is a ~6 MB binary asset from Google and is deliberately NOT stored
# in git (see 1_Sensor_Acquisition/raspberry_pi/docs/OPERATIONS.md).  MediaPipe 1.x needs a .task bundle; the
# legacy `mp.solutions.pose` API no longer exists.
set -euo pipefail

MODEL_DIR="${OASIS_MODEL_DIR:-$HOME/.local/share/oasis-daq/models}"
VARIANT="${1:-lite}"          # lite | full | heavy
BASE="https://storage.googleapis.com/mediapipe-models/pose_landmarker"
URL="${BASE}/pose_landmarker_${VARIANT}/float16/latest/pose_landmarker_${VARIANT}.task"

mkdir -p "$MODEL_DIR"
TARGET="$MODEL_DIR/pose_landmarker_${VARIANT}.task"

if [ -s "$TARGET" ]; then
  echo "already present: $TARGET ($(stat -c%s "$TARGET") bytes)"
  exit 0
fi

echo "downloading $VARIANT model -> $TARGET"
curl -fsSL -o "$TARGET.part" "$URL"
mv "$TARGET.part" "$TARGET"
echo "done: $TARGET ($(stat -c%s "$TARGET") bytes)"
echo
echo "Measured on a Raspberry Pi 5 (Ubuntu 24.04, aarch64):"
echo "  lite : ~13.3 fps standalone, p50 66 ms / p95 94 ms  <- recommended"
echo "  full : significantly slower; only worth it if pose quality is limiting"
