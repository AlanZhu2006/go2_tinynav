#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
SESSION_NAME="${TINYNAV_NAV_SESSION:-tinynav_nav_auto}"
map_path="$ROOT_DIR/output/latest_map"
nav_db_path="$ROOT_DIR/tinynav_temp_nav_auto"
start_rviz="true"
start_go2="true"
display="${TINYNAV_RVIZ_DISPLAY:-${DISPLAY:-:1}}"
rviz_xauthority="${TINYNAV_RVIZ_XAUTHORITY:-/home/nvidia/.Xauthority}"
rviz_config="${TINYNAV_RVIZ_CONFIG:-/tinynav/docs/vis_with_global_map.rviz}"
go2_net_if="${UNITREE_NET_IF:-eth0}"

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
Usage: $0 [--map DIR] [--session NAME] [--no-rviz] [--no-go2] [--go2-net-if IFACE]

Starts the minimal TinyNav navigation stack:
  RealSense if needed
  perception_node
  planning_node
  map_node
  cmd_vel_control
  Go2 cmd_vel bridge
  rviz_goal_to_poi
  static_occupancy_grid_publisher
  RViz

Default map:
  $map_path
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) map_path="$2"; shift 2 ;;
    --session) SESSION_NAME="$2"; shift 2 ;;
    --no-rviz) start_rviz="false"; shift ;;
    --no-go2) start_go2="false"; shift ;;
    --go2-net-if) go2_net_if="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

source_setup
cd "$ROOT_DIR"

if [[ ! -d "$map_path" ]]; then
  echo "Map directory does not exist: $map_path" >&2
  echo "Build one first with: bash $ROOT_DIR/scripts/tinynav_auto_map.sh" >&2
  exit 1
fi

for required in poses.npy intrinsics.npy occupancy_grid.npy occupancy_meta.npy sdf_map.npy; do
  if [[ ! -f "$map_path/$required" ]]; then
    echo "Map is missing $required: $map_path" >&2
    exit 1
  fi
done

camera_running() {
  ros2 node list 2>/dev/null | grep -qx "/camera/camera"
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
  tmux kill-session -t tinynav_usb3_camera >/dev/null 2>&1 || true
  tmux kill-session -t tinynav_auto_map_camera >/dev/null 2>&1 || true
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-45}"
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

ensure_go2_network() {
  if ip -o -4 addr show dev "$go2_net_if" | grep -q "192\\.168\\.123\\."; then
    return 0
  fi

  echo "Adding temporary Go2 address 192.168.123.100/24 to $go2_net_if"
  sudo ip addr add 192.168.123.100/24 dev "$go2_net_if" 2>/dev/null || true
  sudo ip link set "$go2_net_if" up
}

tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
if [[ "$start_go2" == "true" ]]; then
  tmux kill-session -t go2_cmd_bridge >/dev/null 2>&1 || true
  ensure_go2_network
  if ! ping -I "$go2_net_if" -c 1 -W 1 192.168.123.161 >/dev/null 2>&1; then
    echo "Go2 is not reachable at 192.168.123.161 through $go2_net_if." >&2
    echo "Use --no-go2 to start navigation without sending /cmd_vel to the base." >&2
    exit 1
  fi
fi

if [[ "$start_rviz" == "true" && "$display" == ":1" ]]; then
  bash "$ROOT_DIR/scripts/run_visible_vnc.sh"
fi

camera_mode="start"
if camera_running; then
  echo "Found existing RealSense node: /camera/camera"
  if camera_streams_ready 8; then
    camera_mode="existing"
  else
    echo "Existing RealSense node is not publishing all required streams; restarting managed camera session."
    stop_known_camera_sessions
    sleep 4
    if camera_running; then
      echo "A RealSense node is still running outside the managed tmux sessions." >&2
      echo "Stop that node, then rerun this script." >&2
      exit 1
    fi
  fi
fi

if [[ "$camera_mode" == "existing" ]]; then
  tmux new-session -d -s "$SESSION_NAME" -n camera \
    "bash -lc 'echo \"Using existing RealSense node /camera/camera\"; echo \"Do not start another RealSense launch while this is running.\"; sleep infinity'"
else
  tmux new-session -d -s "$SESSION_NAME" -n camera \
    "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && bash scripts/run_realsense_sensor.sh'"
fi

echo "Waiting for camera topics..."
wait_for_topic "/camera/camera/infra1/image_rect_raw" 60
wait_for_topic "/camera/camera/infra2/image_rect_raw" 60
wait_for_topic "/camera/camera/imu" 60
camera_streams_ready 15

tmux new-window -t "$SESSION_NAME" -n perception \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tinynav/core/perception_node.py'"

tmux new-window -t "$SESSION_NAME" -n planning \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tinynav/core/planning_node.py'"

tmux new-window -t "$SESSION_NAME" -n map \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tinynav/core/map_node.py --tinynav_db_path \"$nav_db_path\" --tinynav_map_path \"$map_path\"'"

tmux new-window -t "$SESSION_NAME" -n control \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tinynav/platforms/cmd_vel_control.py'"

if [[ "$start_go2" == "true" ]]; then
  tmux new-window -t "$SESSION_NAME" -n go2-bridge \
    "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && export UNITREE_NET_IF=\"$go2_net_if\" GO2_CMD_TOPIC=/cmd_vel GO2_MAX_VX=0.30 GO2_MAX_VY=0.00 GO2_MAX_WZ=0.70 GO2_MIN_CMD_V=0.10 GO2_MIN_CMD_W=0.20 GO2_REMOTE_PRIORITY=true GO2_REMOTE_TOPIC=rt/lowstate GO2_REMOTE_DEADBAND=0.12 GO2_REMOTE_HOLD_SEC=0.8 GO2_LOG_COMMANDS=true && bash scripts/run_go2_cmd_bridge.sh'"
fi

tmux new-window -t "$SESSION_NAME" -n rviz-goal \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tool/rviz_goal_to_poi.py --tinynav_map_path \"$map_path\" --z 0.0 --marker-z-offset 1.5'"

tmux new-window -t "$SESSION_NAME" -n static-map \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tool/static_occupancy_grid_publisher.py --tinynav-map-path \"$map_path\" --topic /mapping/static_occupancy_grid --frame-id world --z 0.0'"

tmux new-window -t "$SESSION_NAME" -n map-keyframes \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tool/map_keyframe_publisher.py --tinynav-map-path \"$map_path\" --frame-id world'"

if [[ "$start_rviz" == "true" ]]; then
  tmux new-window -t "$SESSION_NAME" -n rviz \
    "bash -lc 'source \"$SETUP_FILE\" && export DISPLAY=\"$display\" XAUTHORITY=\"$rviz_xauthority\" QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 && cd \"$ROOT_DIR\" && ros2 run rviz2 rviz2 -d \"$rviz_config\" & rviz_pid=\$!; sleep 6; wmctrl -r RViz -b remove,maximized_vert,maximized_horz >/dev/null 2>&1 || true; wmctrl -r RViz -e 0,0,0,1280,720 >/dev/null 2>&1 || true; wait \$rviz_pid'"
fi

tmux new-window -t "$SESSION_NAME" -n monitor \
  "bash -lc 'source \"$SETUP_FILE\" && watch -n 1 \"ros2 topic hz /cmd_vel --window 10 2>/dev/null | tail -5; echo; ros2 topic echo --once /mapping/nav_progress 2>/dev/null || true\"'"

echo
echo "TinyNav navigation started."
echo "  session: $SESSION_NAME"
echo "  map:     $map_path"
echo "  go2:     $start_go2"
echo "  rviz:    $rviz_config"
if [[ "$start_go2" == "true" ]]; then
  echo "  go2 net: $go2_net_if -> 192.168.123.161"
fi
echo
echo "Attach:"
echo "  tmux attach -t $SESSION_NAME"
echo
echo "Stop:"
echo "  tmux kill-session -t $SESSION_NAME"
