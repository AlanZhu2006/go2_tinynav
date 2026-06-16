#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${TINYNAV_VNC_DISPLAY_NUM:-1}"
GEOMETRY="${TINYNAV_VNC_GEOMETRY:-1280x720}"
DEPTH="${TINYNAV_VNC_DEPTH:-24}"
if [[ -z "${TINYNAV_VNC_PASSWORD:-}" ]]; then
  echo "TINYNAV_VNC_PASSWORD is not set."
  echo "Start with: TINYNAV_VNC_PASSWORD='<private-password>' $0"
  exit 1
fi
PASSWORD="$TINYNAV_VNC_PASSWORD"
HOST_IP="${TINYNAV_VNC_HOST:-$(hostname -I | awk '{print $1}')}"
PASS_FILE="/home/nvidia/.vnc/passwd"

mkdir -p /home/nvidia/.vnc
printf '%s' "$PASSWORD" | vncpasswd -f > "$PASS_FILE"
chmod 600 "$PASS_FILE"

tmux kill-session -t vnc_real_desktop >/dev/null 2>&1 || true
pkill -f x11vnc >/dev/null 2>&1 || true

vncserver -kill ":$DISPLAY_NUM" >/dev/null 2>&1 || true
pkill -f "Xtigervnc :$DISPLAY_NUM" >/dev/null 2>&1 || true
rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"
rm -f "/home/nvidia/.vnc/tegra-ubuntu:$((5900 + DISPLAY_NUM)).pid"

vncserver ":$DISPLAY_NUM" \
  -geometry "$GEOMETRY" \
  -depth "$DEPTH" \
  -localhost no

echo "Visible VNC started:"
echo "  address: ${HOST_IP:-<jetson-ip>}:$((5900 + DISPLAY_NUM))"
echo "  display: :$DISPLAY_NUM"
echo "  password: set by TINYNAV_VNC_PASSWORD"
echo "  geometry: $GEOMETRY"
