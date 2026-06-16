#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
SESSION_NAME="${TINYNAV_CAMERA_SESSION:-tinynav_usb3_camera}"

timestamp="$(date +%Y%m%d_%H%M%S)"
bag_dir="${XDG_DATA_HOME:-$HOME/.local/share}/tinynav/rosbags/map_record_${timestamp}"
map_dir="$ROOT_DIR/output/map_record_${timestamp}"
play_rate="1.0"
keep_camera="false"
from_bag=""
clean_temp="true"

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

usage() {
  cat <<EOF
Usage: $0 [--bag-dir DIR] [--map-dir DIR] [--play-rate RATE] [--keep-camera] [--keep-temp]
       $0 --from-bag DIR [--map-dir DIR] [--play-rate RATE] [--keep-temp]

Records a RealSense mapping bag, then builds a TinyNav map from it.
Stop recording with Ctrl-C once you have walked the mapping trajectory.
With --from-bag, skips recording and only builds a map from an existing bag.

Outputs:
  bag:      $bag_dir
  map:      $map_dir
  symlink:  $ROOT_DIR/output/latest_map

By default, successful map builds remove TinyNav runtime temp DBs:
  $ROOT_DIR/tinynav_temp
  $ROOT_DIR/tinynav_temp_gpu_current
  $ROOT_DIR/tinynav_temp_nav_auto
Use --keep-temp to keep them for debugging.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-dir) bag_dir="$2"; shift 2 ;;
    --from-bag) from_bag="$2"; bag_dir="$2"; shift 2 ;;
    --map-dir) map_dir="$2"; shift 2 ;;
    --play-rate) play_rate="$2"; shift 2 ;;
    --keep-camera) keep_camera="true"; shift ;;
    --keep-temp) clean_temp="false"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

source_setup
cd "$ROOT_DIR"

camera_started_here="false"
perception_pid=""

camera_running() {
  ros2 node list 2>/dev/null | grep -qx "/camera/camera"
}

start_camera() {
  echo "Starting USB3 RealSense in tmux session: $SESSION_NAME"
  tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
  tmux new-session -d -s "$SESSION_NAME" \
    "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && bash scripts/run_realsense_sensor.sh'"
  camera_started_here="true"
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-30}"
  local start
  start="$(date +%s)"
  until ros2 topic list 2>/dev/null | grep -qx "$topic"; do
    if (( "$(date +%s)" - start >= timeout_s )); then
      echo "Timed out waiting for $topic" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_message() {
  local topic="$1"
  local timeout_s="${2:-8}"
  timeout "$timeout_s" ros2 topic echo --once "$topic" >/dev/null 2>&1
}

camera_streams_ready() {
  local timeout_s="${1:-8}"
  local topic
  local required_topics=(
    "/camera/camera/infra1/image_rect_raw"
    "/camera/camera/infra2/image_rect_raw"
    "/camera/camera/depth/image_rect_raw"
    "/camera/camera/color/image_raw"
    "/camera/camera/infra1/camera_info"
    "/camera/camera/infra2/camera_info"
    "/camera/camera/color/camera_info"
    "/camera/camera/imu"
  )

  for topic in "${required_topics[@]}"; do
    echo "Checking stream: $topic"
    if ! wait_for_message "$topic" "$timeout_s"; then
      echo "No message received from $topic within ${timeout_s}s" >&2
      return 1
    fi
  done
}

stop_known_camera_sessions() {
  tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
  tmux kill-session -t tinynav_usb3_camera >/dev/null 2>&1 || true
  tmux kill-session -t tinynav_auto_map_camera >/dev/null 2>&1 || true
}

realsense_process_running() {
  pgrep -f 'realsense2_camera|rs_launch.py|run_realsense_sensor.sh' >/dev/null 2>&1
}

wait_for_camera_node_gone() {
  local timeout_s="${1:-12}"
  local start
  start="$(date +%s)"

  until ! camera_running; do
    if (( "$(date +%s)" - start >= timeout_s )); then
      if realsense_process_running; then
        return 1
      fi

      echo "ROS still lists /camera/camera, but no RealSense process is running; refreshing ROS daemon discovery."
      ros2 daemon stop >/dev/null 2>&1 || true
      sleep 2
      ros2 daemon start >/dev/null 2>&1 || true
      sleep 2
      ! camera_running
      return
    fi
    sleep 1
  done
}

ensure_camera_streams() {
  if camera_running; then
    echo "Using existing RealSense node: /camera/camera"
    if camera_streams_ready 8; then
      return 0
    fi

    echo "Existing RealSense node is not publishing all required streams; restarting managed camera session."
    stop_known_camera_sessions
    if ! wait_for_camera_node_gone 15; then
      echo "A RealSense node is still running outside the managed tmux sessions." >&2
      echo "Stop that node, then rerun this script." >&2
      return 1
    fi
  fi

  start_camera
  wait_for_topic "/camera/camera/infra1/image_rect_raw" 60
  wait_for_topic "/camera/camera/infra2/image_rect_raw" 60
  wait_for_topic "/camera/camera/imu" 60
  camera_streams_ready 15
}

stop_camera_if_owned() {
  if [[ "$camera_started_here" == "true" && "$keep_camera" != "true" ]]; then
    tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
  fi
}

stop_perception_if_running() {
  if [[ -n "$perception_pid" ]]; then
    kill "$perception_pid" >/dev/null 2>&1 || true
    wait "$perception_pid" >/dev/null 2>&1 || true
    perception_pid=""
  fi
}

cleanup() {
  stop_perception_if_running
  stop_camera_if_owned
}

trap cleanup EXIT

cleanup_runtime_temp() {
  if [[ "$clean_temp" != "true" ]]; then
    echo "Keeping TinyNav runtime temp DBs because --keep-temp was set."
    return 0
  fi

  local temp_dirs=(
    "$ROOT_DIR/tinynav_temp"
    "$ROOT_DIR/tinynav_temp_gpu_current"
    "$ROOT_DIR/tinynav_temp_nav_auto"
  )
  local dir

  echo
  echo "Cleaning TinyNav runtime temp DBs..."
  for dir in "${temp_dirs[@]}"; do
    if [[ -e "$dir" ]]; then
      du -sh "$dir" 2>/dev/null || true
      rm -rf --one-file-system "$dir"
      echo "  removed: $dir"
    fi
  done
}

mkdir -p "$(dirname "$bag_dir")" "$(dirname "$map_dir")" "$ROOT_DIR/output"

if [[ -z "$from_bag" ]]; then
  echo "Checking RealSense streams..."
  ensure_camera_streams

  echo
  echo "Recording mapping bag:"
  echo "  $bag_dir"
  echo "Move through the mapping area now. Press Ctrl-C once to stop recording and start map building."
  echo

  set +e
  bash "$ROOT_DIR/scripts/run_rosbag_record.sh" --output "$bag_dir"
  record_status=$?
  set -e

  if [[ "$record_status" -ne 0 && "$record_status" -ne 130 && "$record_status" -ne 143 ]]; then
    echo "rosbag recording failed with status $record_status" >&2
    exit "$record_status"
  fi
else
  if [[ ! -d "$bag_dir" ]]; then
    echo "Bag directory does not exist: $bag_dir" >&2
    exit 1
  fi
  echo "Building from existing bag:"
  echo "  $bag_dir"
fi

echo
echo "Recording stopped. Bag info:"
ros2 bag info "$bag_dir" || true

echo
python3 "$ROOT_DIR/tool/validate_tinynav_bag.py" --bag "$bag_dir"

if [[ "$keep_camera" != "true" ]]; then
  echo "Stopping RealSense before offline map build to avoid duplicate /camera publishers."
  stop_known_camera_sessions
  camera_started_here="false"
  if ! wait_for_camera_node_gone 15; then
    echo "A /camera/camera node is still running outside the managed tmux sessions." >&2
    echo "Stop that camera node before offline map building; duplicate /camera publishers break timestamp sync." >&2
    exit 1
  fi
else
  echo "Keeping RealSense running; make sure no offline bag is being played into the same /camera topics."
fi

echo
echo "Building TinyNav map:"
echo "  $map_dir"
rm -rf "$map_dir"

echo "Starting perception_node for offline odometry/keyframes..."
(
  source_setup
  cd "$ROOT_DIR"
  uv run python /tinynav/tinynav/core/perception_node.py
) &
perception_pid=$!
sleep 5

uv run python /tinynav/tinynav/core/build_map_node.py \
  --bag_file "$bag_dir" \
  --map_save_path "$map_dir" \
  --play_rate "$play_rate"
stop_perception_if_running

for required in poses.npy intrinsics.npy occupancy_grid.npy occupancy_meta.npy sdf_map.npy; do
  if [[ ! -f "$map_dir/$required" ]]; then
    echo "Map build did not produce $required; latest_map was not updated." >&2
    exit 1
  fi
done

ln -sfn "$map_dir" "$ROOT_DIR/output/latest_map"
cleanup_runtime_temp

echo
echo "Map build finished."
echo "  map:     $map_dir"
echo "  latest:  $ROOT_DIR/output/latest_map"
echo
echo "Start navigation with:"
echo "  bash $ROOT_DIR/scripts/tinynav_auto_nav.sh --map $ROOT_DIR/output/latest_map"
