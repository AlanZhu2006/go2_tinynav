#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
SESSION_NAME="${TINYNAV_BASELINE_NAV_SESSION:-tinynav_baseline_nav}"

map_path="${TINYNAV_MAP_PATH:-$ROOT_DIR/output/latest_map}"
rviz_config="${TINYNAV_RVIZ_CONFIG:-/tinynav/docs/vis.rviz}"
display="${TINYNAV_RVIZ_DISPLAY:-${DISPLAY:-:1}}"
rviz_xauthority="${TINYNAV_RVIZ_XAUTHORITY:-/home/nvidia/.Xauthority}"
start_rviz="true"
start_go2="false"
go2_net_if="${UNITREE_NET_IF:-eth0}"

usage() {
  cat <<EOF
Usage: $0 [--map DIR] [--no-rviz] [--with-go2] [--go2-net-if IFACE]

Official-baseline-style navigation without modifying upstream scripts.
This copies the node layout from scripts/run_navigation.sh:
  perception_node
  planning_node
  map_node
  cmd_vel_control
  RViz docs/vis.rviz
  pub_pois.py

Unlike tinynav_auto_nav.sh, this script does not start RealSense automatically.
Start the camera first if /camera/camera is not already running.

Default map:
  $map_path
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) map_path="$2"; shift 2 ;;
    --no-rviz) start_rviz="false"; shift ;;
    --with-go2) start_go2="true"; shift ;;
    --go2-net-if) go2_net_if="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if [[ ! -d "$map_path" ]]; then
  echo "Map directory does not exist: $map_path" >&2
  exit 1
fi

for required in poses.npy intrinsics.npy occupancy_grid.npy occupancy_meta.npy sdf_map.npy; do
  if [[ ! -f "$map_path/$required" ]]; then
    echo "Map is missing $required: $map_path" >&2
    exit 1
  fi
done

if [[ "$start_go2" == "true" ]]; then
  if ! ip -o -4 addr show dev "$go2_net_if" | grep -q "192\\.168\\.123\\."; then
    echo "Adding temporary Go2 address 192.168.123.100/24 to $go2_net_if"
    sudo ip addr add 192.168.123.100/24 dev "$go2_net_if" 2>/dev/null || true
    sudo ip link set "$go2_net_if" up
  fi

  if ! ping -I "$go2_net_if" -c 1 -W 1 192.168.123.161 >/dev/null 2>&1; then
    echo "Go2 is not reachable at 192.168.123.161 through $go2_net_if." >&2
    exit 1
  fi
fi

q_setup="$(printf '%q' "$SETUP_FILE")"
q_root="$(printf '%q' "$ROOT_DIR")"
q_map="$(printf '%q' "$map_path")"
q_rviz="$(printf '%q' "$rviz_config")"
q_display="$(printf '%q' "$display")"
q_xauth="$(printf '%q' "$rviz_xauthority")"
q_go2_if="$(printf '%q' "$go2_net_if")"

tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true

if [[ "$start_rviz" == "true" ]]; then
  rviz_cmd="source ${q_setup} && export DISPLAY=${q_display} XAUTHORITY=${q_xauth} QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 && cd ${q_root} && ros2 run rviz2 rviz2 -d ${q_rviz}"
else
  rviz_cmd="echo RViz disabled by --no-rviz; sleep infinity"
fi

# This intentionally mirrors upstream scripts/run_navigation.sh:
#   perception, planning, map, cmd_vel_control, RViz, and pub_pois.
tmux new-session -d -s "$SESSION_NAME" \; \
  split-window -h \; \
  split-window -v \; \
  select-pane -t 0 \; split-window -v \; \
  select-pane -t 3 \; split-window -v \; \
  select-pane -t 4 \; split-window -v \; \
  select-pane -t 0 \; send-keys "source ${q_setup} && cd ${q_root} && uv run python /tinynav/tinynav/core/perception_node.py" C-m \; \
  select-pane -t 1 \; send-keys "source ${q_setup} && cd ${q_root} && uv run python /tinynav/tinynav/core/planning_node.py" C-m \; \
  select-pane -t 2 \; send-keys "source ${q_setup} && cd ${q_root} && uv run python /tinynav/tinynav/core/map_node.py --tinynav_map_path ${q_map}" C-m \; \
  select-pane -t 3 \; send-keys "source ${q_setup} && cd ${q_root} && uv run python /tinynav/tinynav/platforms/cmd_vel_control.py" C-m \; \
  select-pane -t 4 \; send-keys "$rviz_cmd" C-m \; \
  select-pane -t 5 \; send-keys "source ${q_setup} && cd ${q_root} && sleep 3 && uv run python /tinynav/tool/pub_pois.py --tinynav_map_path ${q_map}" C-m

if [[ "$start_go2" == "true" ]]; then
  tmux new-window -t "$SESSION_NAME" -n go2-bridge \
    "bash -lc 'source ${q_setup} && cd ${q_root} && export UNITREE_NET_IF=${q_go2_if} GO2_CMD_TOPIC=/cmd_vel GO2_MAX_VX=0.30 GO2_MAX_VY=0.00 GO2_MAX_WZ=0.70 GO2_MIN_CMD_V=0.10 GO2_MIN_CMD_W=0.20 GO2_LOG_COMMANDS=true && bash scripts/run_go2_cmd_bridge.sh'"
fi

echo "Official-baseline navigation started."
echo "  session: $SESSION_NAME"
echo "  map:     $map_path"
echo "  rviz:    $start_rviz ($rviz_config)"
echo "  go2:     $start_go2"
echo
echo "Attach:"
echo "  tmux attach -t $SESSION_NAME"
echo
echo "Stop:"
echo "  tmux kill-session -t $SESSION_NAME"
