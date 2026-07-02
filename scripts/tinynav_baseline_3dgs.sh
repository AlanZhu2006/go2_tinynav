#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"

map_path="${TINYNAV_MAP_PATH:-$ROOT_DIR/output/latest_map}"
output_path="${TINYNAV_3DGS_OUTPUT:-$map_path}"
install_deps="false"

usage() {
  cat <<EOF
Usage: $0 [--map DIR] [--output DIR] [--install-deps]

Official-baseline-style 3DGS generation without modifying upstream scripts.
This copies the commands from scripts/run_3dgs_generation.sh, but parameterizes
the hard-coded map path:
  convert_to_nerf_format.py
  ns-train splatfacto
  ns-export gaussian-splat

Default:
  --map     $map_path
  --output  same as --map
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) map_path="$2"; output_path="${TINYNAV_3DGS_OUTPUT:-$2}"; shift 2 ;;
    --output) output_path="$2"; shift 2 ;;
    --install-deps) install_deps="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if [[ ! -d "$map_path" ]]; then
  echo "Map directory does not exist: $map_path" >&2
  exit 1
fi

source_setup() {
  local had_nounset=0
  case $- in
    *u*) had_nounset=1; set +u ;;
  esac
  source "$SETUP_FILE"
  if [[ "$had_nounset" == "1" ]]; then
    set -u
  fi
}

source_setup
cd "$ROOT_DIR"

if [[ "$install_deps" == "true" ]]; then
  uv pip install ".[3dgs]"
fi

uv run python tool/convert_to_nerf_format.py --map-dir "$map_path"

MAX_JOBS=1 uv run ns-train splatfacto \
  --output-dir "$output_path" \
  --experiment-name experiment \
  --method-name splatfacto \
  --timestamp 0 \
  --pipeline.model.cull_alpha_thresh=0.005 \
  --pipeline.model.use_scale_regularization True \
  --viewer.quit-on-train-completion True \
  nerfstudio-data \
  --data "$map_path" \
  --center-method none \
  --auto-scale-poses False \
  --orientation_method none

uv run ns-export gaussian-splat \
  --load-config "$output_path/experiment/splatfacto/0/config.yml" \
  --output-dir "$output_path"

echo "3DGS export finished:"
echo "  output: $output_path"
