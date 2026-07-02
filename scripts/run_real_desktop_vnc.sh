#!/usr/bin/env bash
set -euo pipefail

DISPLAY_ID="${TINYNAV_VNC_DISPLAY:-:0}"
XAUTHORITY_FILE="${TINYNAV_VNC_XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
VNC_PORT="${TINYNAV_VNC_PORT:-5901}"
VNC_PASSWORD="${TINYNAV_VNC_PASSWORD:-nvidia}"
VNC_RESOLUTION="${TINYNAV_VNC_RESOLUTION:-1280x720}"
PASS_FILE="${TINYNAV_VNC_PASS_FILE:-/home/nvidia/.vnc/x11vnc.pass}"
LOG_FILE="${TINYNAV_VNC_LOG_FILE:-/home/nvidia/.vnc/x11vnc-real.log}"
SESSION_NAME="${TINYNAV_VNC_SESSION:-vnc_real_desktop}"

mkdir -p "$(dirname "$PASS_FILE")"
x11vnc -storepasswd "$VNC_PASSWORD" "$PASS_FILE" >/dev/null
chmod 600 "$PASS_FILE"

vncserver -kill :1 >/dev/null 2>&1 || true
rm -f /home/nvidia/.vnc/tegra-ubuntu:5901.pid

DISPLAY="$DISPLAY_ID" XAUTHORITY="$XAUTHORITY_FILE" \
  xrandr --fb "$VNC_RESOLUTION" >/dev/null 2>&1 || true

tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
tmux new-session -d -s "$SESSION_NAME" -n x11vnc \
  "bash -lc 'DISPLAY=\"$DISPLAY_ID\" XAUTHORITY=\"$XAUTHORITY_FILE\" x11vnc -display \"$DISPLAY_ID\" -auth \"$XAUTHORITY_FILE\" -rfbauth \"$PASS_FILE\" -rfbport \"$VNC_PORT\" -forever -shared -noxdamage -repeat -o \"$LOG_FILE\"'"

sleep 1
echo "Real desktop VNC started:"
echo "  display:    $DISPLAY_ID"
echo "  xauthority: $XAUTHORITY_FILE"
echo "  port:       $VNC_PORT"
echo "  password:   $VNC_PASSWORD"
echo "  resolution: $(DISPLAY="$DISPLAY_ID" XAUTHORITY="$XAUTHORITY_FILE" xdpyinfo 2>/dev/null | awk '/dimensions:/ {print $2; exit}')"
