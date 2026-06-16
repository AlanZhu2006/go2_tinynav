#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
BRIDGE_SCRIPT="${GO2_CMD_BRIDGE_SCRIPT:-$ROOT_DIR/tool/go2_cmd_bridge.py}"
CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/nvidia/twork/cyclonedds/install}"

UNITREE_NET_IF="${UNITREE_NET_IF:-eth0}"
UNITREE_SDK2PY_PATH="${UNITREE_SDK2PY_PATH:-}"
GO2_CMD_TOPIC="${GO2_CMD_TOPIC:-/cmd_vel}"
GO2_MAX_VX="${GO2_MAX_VX:-0.45}"
GO2_MAX_VY="${GO2_MAX_VY:-0.25}"
GO2_MAX_WZ="${GO2_MAX_WZ:-0.80}"
GO2_X_SIGN="${GO2_X_SIGN:-1.0}"
GO2_Y_SIGN="${GO2_Y_SIGN:-1.0}"
GO2_WZ_SIGN="${GO2_WZ_SIGN:-1.0}"
GO2_SWAP_XY="${GO2_SWAP_XY:-false}"
GO2_DEADBAND_V="${GO2_DEADBAND_V:-0.01}"
GO2_DEADBAND_W="${GO2_DEADBAND_W:-0.02}"
GO2_MIN_CMD_V="${GO2_MIN_CMD_V:-0.10}"
GO2_MIN_CMD_W="${GO2_MIN_CMD_W:-0.20}"
GO2_SEND_ZERO_WHEN_IDLE="${GO2_SEND_ZERO_WHEN_IDLE:-false}"
GO2_REMOTE_PRIORITY="${GO2_REMOTE_PRIORITY:-false}"
GO2_LOG_COMMANDS="${GO2_LOG_COMMANDS:-true}"
GO2_LOG_INTERVAL_SEC="${GO2_LOG_INTERVAL_SEC:-0.2}"

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

if [[ ! -f "$BRIDGE_SCRIPT" ]]; then
  echo "Go2 bridge script not found: $BRIDGE_SCRIPT" >&2
  exit 1
fi

if ! ip -o -4 addr show dev "$UNITREE_NET_IF" >/dev/null 2>&1; then
  echo "No IPv4 address on $UNITREE_NET_IF." >&2
  echo "For the current Go2 setup, run this transient network setup first:" >&2
  echo "  sudo ip addr add 192.168.123.100/24 dev $UNITREE_NET_IF" >&2
  echo "  sudo ip link set $UNITREE_NET_IF up" >&2
  exit 1
fi

source_setup
cd "$ROOT_DIR"

export CYCLONEDDS_HOME
export CMAKE_PREFIX_PATH="$CYCLONEDDS_HOME${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "Starting README Go2 cmd_vel bridge"
echo "  bridge:    $BRIDGE_SCRIPT"
echo "  topic:     $GO2_CMD_TOPIC"
echo "  interface: $UNITREE_NET_IF"
echo "  cyclone:   $CYCLONEDDS_HOME"
echo "  limits:    vx=$GO2_MAX_VX vy=$GO2_MAX_VY wz=$GO2_MAX_WZ"
echo "  floors:    v=$GO2_MIN_CMD_V w=$GO2_MIN_CMD_W"

exec uv run --extra unitree python "$BRIDGE_SCRIPT" \
  --net-if "$UNITREE_NET_IF" \
  --sdk-path "$UNITREE_SDK2PY_PATH" \
  --ros-args \
  -p cmd_vel_topic:="$GO2_CMD_TOPIC" \
  -p max_vx:="$GO2_MAX_VX" \
  -p max_vy:="$GO2_MAX_VY" \
  -p max_wz:="$GO2_MAX_WZ" \
  -p x_sign:="$GO2_X_SIGN" \
  -p y_sign:="$GO2_Y_SIGN" \
  -p wz_sign:="$GO2_WZ_SIGN" \
  -p swap_xy:="$GO2_SWAP_XY" \
  -p deadband_v:="$GO2_DEADBAND_V" \
  -p deadband_w:="$GO2_DEADBAND_W" \
  -p min_cmd_v:="$GO2_MIN_CMD_V" \
  -p min_cmd_w:="$GO2_MIN_CMD_W" \
  -p enabled:=true \
  -p send_zero_when_idle:="$GO2_SEND_ZERO_WHEN_IDLE" \
  -p remote_priority:="$GO2_REMOTE_PRIORITY" \
  -p log_commands:="$GO2_LOG_COMMANDS" \
  -p log_interval_sec:="$GO2_LOG_INTERVAL_SEC"
