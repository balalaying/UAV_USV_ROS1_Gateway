#!/usr/bin/env bash
set -euo pipefail

# Copy only the external preview model assets into /tmp and expand the source
# one-tile FFT model to a 5 x 5 tiled surface.  No GPL source is copied into
# the repository itself.

ASV_WAVE_ROOT="${ASV_WAVE_ROOT:-/tmp/asv_wave_sim}"
OUTPUT_ROOT="${ASV_WAVE_MODEL_DIR:-/tmp/UAV_USV_asv_fft_models}"
SOURCE_MODEL="${ASV_WAVE_ROOT}/gz-waves-models/world_models/waves"
OUTPUT_MODEL="${OUTPUT_ROOT}/waves"

if [[ ! -f "${SOURCE_MODEL}/model.sdf" ]]; then
  echo "asv_wave_sim model not found: ${SOURCE_MODEL}/model.sdf" >&2
  exit 2
fi

mkdir -p "${OUTPUT_MODEL}"
cp -a "${SOURCE_MODEL}/." "${OUTPUT_MODEL}/"
sed \
  -e 's/<tiles_x>-0 0<\//<tiles_x>-2 2<\//' \
  -e 's/<tiles_y>-0 0<\//<tiles_y>-2 2<\//' \
  "${SOURCE_MODEL}/model.sdf" > "${OUTPUT_MODEL}/model.sdf"

echo "Prepared tiled FFT preview model: ${OUTPUT_MODEL}"
