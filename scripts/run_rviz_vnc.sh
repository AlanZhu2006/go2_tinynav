#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"

display="${TINYNAV_RVIZ_DISPLAY:-:1}"
if [[ -n "${TINYNAV_RVIZ_XAUTHORITY:-}" ]]; then
  xauthority="$TINYNAV_RVIZ_XAUTHORITY"
elif [[ -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  xauthority="/run/user/$(id -u)/gdm/Xauthority"
else
  xauthority="/home/nvidia/.Xauthority"
fi

rviz_config="${TINYNAV_RVIZ_CONFIG:-/tinynav/docs/vis_with_global_map.rviz}"
log_dir="${TINYNAV_LOG_DIR:-$ROOT_DIR/logs}"
restart="${TINYNAV_RVIZ_RESTART:-true}"

mkdir -p "$log_dir"
log_file="$log_dir/rviz.log"

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

wait_for_display() {
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if DISPLAY="$display" XAUTHORITY="$xauthority" xrandr >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

source_setup
cd "$ROOT_DIR"

export DISPLAY="$display"
export XAUTHORITY="$xauthority"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

echo "RViz launcher"
echo "  display:    $DISPLAY"
echo "  xauthority: $XAUTHORITY"
echo "  config:     $rviz_config"
echo "  log:        $log_file"
echo "  restart:    $restart"

if ! wait_for_display; then
  echo "RViz display is not reachable: DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY" | tee -a "$log_file" >&2
  sleep infinity
fi

while true; do
  {
    echo
    echo "===== RViz start $(date -Is) ====="
    echo "DISPLAY=$DISPLAY"
    echo "XAUTHORITY=$XAUTHORITY"
    echo "config=$rviz_config"
  } >> "$log_file"

  (
    sleep 6
    wmctrl -r RViz -b remove,maximized_vert,maximized_horz >/dev/null 2>&1 || true
    wmctrl -r RViz -e 0,0,0,1280,720 >/dev/null 2>&1 || true
  ) &

  ros2 run rviz2 rviz2 -d "$rviz_config" 2>&1 | tee -a "$log_file"
  rviz_status="${PIPESTATUS[0]}"
  echo "===== RViz exit $(date -Is), status=$rviz_status =====" | tee -a "$log_file"

  if [[ "$restart" != "true" ]]; then
    break
  fi
  sleep 3
done

sleep infinity
