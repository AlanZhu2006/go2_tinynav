#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
SESSION_NAME="${TINYNAV_BASELINE_BUILD_SESSION:-tinynav_baseline_build_map}"

timestamp="$(date +%Y%m%d_%H%M%S)"
bag_path="${TINYNAV_BAG_PATH:-}"
map_save_path="${TINYNAV_MAP_SAVE_PATH:-$ROOT_DIR/output/baseline_map_${timestamp}}"
mode="${TINYNAV_BUILD_MODE:-perception}"
play_rate="${TINYNAV_PLAY_RATE:-1.0}"
rviz_config="${TINYNAV_RVIZ_CONFIG:-/tinynav/docs/vis.rviz}"
display="${TINYNAV_RVIZ_DISPLAY:-${DISPLAY:-:1}}"
rviz_xauthority="${TINYNAV_RVIZ_XAUTHORITY:-/home/nvidia/.Xauthority}"
start_rviz="true"
update_latest="true"

usage() {
  cat <<EOF
Usage: $0 --bag DIR [--map-dir DIR] [--mode perception|looper_direct] [--play-rate RATE] [--no-rviz] [--no-latest]

Official-baseline-style offline map build without modifying upstream scripts.
This copies the node layout from scripts/run_rosbag_build_map.sh:
  build_map_node + perception/looper source + RViz docs/vis.rviz

Required:
  --bag DIR        Existing rosbag directory

Defaults:
  --map-dir        $map_save_path
  --mode           perception
  --play-rate      1.0
  RViz config      $rviz_config

Output:
  map directory    --map-dir value
  latest symlink   $ROOT_DIR/output/latest_map, unless --no-latest is set
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag|--bag-dir) bag_path="$2"; shift 2 ;;
    --map-dir|--map-save-path) map_save_path="$2"; shift 2 ;;
    --mode) mode="$2"; shift 2 ;;
    --play-rate) play_rate="$2"; shift 2 ;;
    --no-rviz) start_rviz="false"; shift ;;
    --no-latest) update_latest="false"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if [[ -z "$bag_path" ]]; then
  echo "Missing --bag DIR" >&2
  usage >&2
  exit 1
fi

if [[ ! -d "$bag_path" ]]; then
  echo "Bag directory does not exist: $bag_path" >&2
  exit 1
fi

case "$mode" in
  perception)
    source_cmd="uv run python /tinynav/tinynav/core/perception_node.py"
    ;;
  looper_direct)
    source_cmd="uv run python /tinynav/tool/looper_bridge_node.py"
    ;;
  *)
    echo "Unsupported mode: $mode" >&2
    echo "Use perception or looper_direct." >&2
    exit 1
    ;;
esac

mkdir -p "$(dirname "$map_save_path")" "$ROOT_DIR/output"

q_setup="$(printf '%q' "$SETUP_FILE")"
q_root="$(printf '%q' "$ROOT_DIR")"
q_bag="$(printf '%q' "$bag_path")"
q_map="$(printf '%q' "$map_save_path")"
q_rate="$(printf '%q' "$play_rate")"
q_rviz="$(printf '%q' "$rviz_config")"
q_display="$(printf '%q' "$display")"
q_xauth="$(printf '%q' "$rviz_xauthority")"

latest_cmd=""
if [[ "$update_latest" == "true" ]]; then
  latest_cmd="&& for f in poses.npy intrinsics.npy occupancy_grid.npy occupancy_meta.npy sdf_map.npy; do test -f ${q_map}/\$f; done && ln -sfn ${q_map} ${q_root}/output/latest_map && echo latest_map updated: ${q_root}/output/latest_map"
fi

build_cmd="source ${q_setup} && cd ${q_root} && rm -rf ${q_map} && uv run python /tinynav/tinynav/core/build_map_node.py --map_save_path ${q_map} --bag_file ${q_bag} --play_rate ${q_rate} ${latest_cmd}"
source_pane_cmd="source ${q_setup} && cd ${q_root} && ${source_cmd}"
if [[ "$start_rviz" == "true" ]]; then
  rviz_cmd="source ${q_setup} && export DISPLAY=${q_display} XAUTHORITY=${q_xauth} QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 && cd ${q_root} && ros2 run rviz2 rviz2 -d ${q_rviz}"
else
  rviz_cmd="echo RViz disabled by --no-rviz; sleep infinity"
fi

tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true

# This intentionally mirrors upstream scripts/run_rosbag_build_map.sh:
#   tmux layout with build_map_node, perception/looper source, and RViz.
tmux new-session -d -s "$SESSION_NAME" \; \
  split-window -h \; \
  split-window -v \; \
  select-pane -t 0 \; split-window -v \; \
  select-pane -t 1 \; send-keys "$build_cmd" C-m \; \
  select-pane -t 2 \; send-keys "$source_pane_cmd" C-m \; \
  select-pane -t 3 \; send-keys "$rviz_cmd" C-m

echo "Official-baseline map build started."
echo "  session: $SESSION_NAME"
echo "  bag:     $bag_path"
echo "  map:     $map_save_path"
echo "  mode:    $mode"
echo "  rviz:    $start_rviz ($rviz_config)"
echo
echo "Attach:"
echo "  tmux attach -t $SESSION_NAME"
