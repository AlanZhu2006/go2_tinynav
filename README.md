# TinyNav + RealSense D435i + Unitree Go2 Deployment

Last updated: 2026-06-16

This document records the current Jetson deployment used for TinyNav mapping,
localization, RViz inspection, and Unitree Go2 `/cmd_vel` execution.

## Machine State

- User: `nvidia`
- Workspace: `/home/nvidia/twork`
- TinyNav repository: `/home/nvidia/twork/tinynav`
- Compatibility symlink: `/tinynav -> /home/nvidia/twork/tinynav`
- ROS: ROS 2 Humble
- Camera: Intel RealSense D435i
- Current camera firmware: `5.17.0.10`
- Current camera transport: USB 3.x, verified at `5000M`
- VNC endpoint: use the Jetson LAN IP plus port `5901`
- VNC password: set locally with `TINYNAV_VNC_PASSWORD`; do not commit it

Environment entry point:

```bash
source /home/nvidia/twork/tinynav_setup.bash
cd /home/nvidia/twork/tinynav
```

The setup file sources ROS Humble, the local `message_filters_ws`, the local
`realsense_ws`, TensorRT tools, and the GTSAM Python binding path.

## Repository Additions

The deployment keeps TinyNav core logic close to the upstream baseline. The
local additions are operational scripts and diagnostics:

- `scripts/tinynav_auto_map.sh`: record a RealSense bag, validate it, build a map, update `output/latest_map`, and clean runtime caches.
- `scripts/tinynav_auto_nav.sh`: start the minimal navigation stack from `output/latest_map`.
- `scripts/run_realsense_sensor.sh`: RealSense launch with D435i USB3 TinyNav streams.
- `scripts/run_go2_cmd_bridge.sh`: bridge ROS `/cmd_vel` to the previous Go2 SDK2 bridge.
- `scripts/run_keyboard_teleop.sh`: keyboard `/cmd_vel` test through the same bridge path.
- `scripts/run_visible_vnc.sh`: visible TigerVNC desktop for RViz on headless Jetson.
- `tool/rviz_goal_to_poi.py`: convert RViz `2D Goal Pose` into TinyNav `pois.json` plus `/mapping/cmd_pois`.
- `tool/static_occupancy_grid_publisher.py`: publish the saved TinyNav map as `/mapping/static_occupancy_grid`.
- `tool/map_keyframe_publisher.py`: publish saved map keyframe trajectory and markers for localization diagnostics.
- `tool/keyboard_cmd_vel.py`: keyboard teleop node.
- `tool/go2_cmd_bridge.py`: ROS `/cmd_vel` to Unitree SDK2 `SportClient.Move()` bridge, adapted from the previous Go2 navigation project.
- `tool/validate_tinynav_bag.py`: validate required TinyNav bag streams and timestamp gaps.
- `docs/vis_with_global_map.rviz`: RViz config with global map, map keyframes, current pose, relocalization, local planning, and camera panels.

## Large Files Policy

Do not commit runtime data or generated engines:

- `output/`
- `tinynav_temp_nav_auto/`
- `tinynav_db/`
- `tinynav_temp/`
- `*.db`
- `*.plan`
- `*.engine`
- rosbags under `/home/nvidia/.local/share/tinynav/rosbags`

Large model files should be obtained from the upstream TinyNav repository or
generated locally with TensorRT. Current local maps and bags are deployment
artifacts, not source files.

Excluded model artifacts:

- `tinynav/models/dinov2_base_224x224_fp16.onnx`
- `tinynav/models/lightglue_fp16.onnx`
- `tinynav/models/superpoint_fp16_dynamic.onnx`
- `tinynav/models/retinify_0_1_5_dynamic.onnx`
- `tinynav/models/foundation_stereo_11-33-40_256x320_4.onnx`
- all generated `*.plan` / `*.engine` TensorRT files

Use the upstream TinyNav model sources as the canonical source for these large
files:

```text
https://github.com/UniflexAI/tinynav
```

On the Jetson deployment machine, the generated aarch64 TensorRT engines live
under `/home/nvidia/twork/tinynav/tinynav/models`, but they are intentionally not
part of this repository.

## RealSense Configuration

The current stable camera launch is:

```bash
source /home/nvidia/twork/tinynav_setup.bash
cd /home/nvidia/twork/tinynav
bash scripts/run_realsense_sensor.sh
```

Equivalent launch parameters:

```bash
ros2 launch realsense2_camera rs_launch.py \
  initial_reset:=true \
  tf_publish_rate:=1.0 \
  publish_tf:=true \
  enable_depth:=true \
  enable_color:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  depth_module.depth_profile:=848x480x30 \
  depth_module.infra_profile:=848x480x30 \
  rgb_camera.color_profile:=848x480x30
```

The mapping scripts validate these required streams:

- `/camera/camera/infra1/image_rect_raw`
- `/camera/camera/infra2/image_rect_raw`
- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/color/image_raw`
- `/camera/camera/infra1/camera_info`
- `/camera/camera/infra2/camera_info`
- `/camera/camera/color/camera_info`
- `/camera/camera/imu`
- `/tf_static`

Timestamp gaps on image streams cause bad maps. The auto-map script validates
the bag before building.

## Mapping Flow

Run:

```bash
source /home/nvidia/twork/tinynav_setup.bash
cd /home/nvidia/twork/tinynav
bash scripts/tinynav_auto_map.sh
```

The script:

1. Starts or reuses a valid RealSense node.
2. Records a timestamped bag under `/home/nvidia/.local/share/tinynav/rosbags`.
3. Validates required topics and timestamp continuity.
4. Stops RealSense before offline build to avoid duplicate `/camera` publishers.
5. Runs TinyNav map building.
6. Updates `/home/nvidia/twork/tinynav/output/latest_map`.
7. Removes runtime temp DBs after a successful build.

Good map collection practice:

- Walk slowly.
- Use clear translational motion; do not only rotate in place.
- Keep textured objects in view.
- Avoid glass, mirrors, pure white walls, overexposure, and fast shakes.
- Include a loop closure when possible.

## Navigation Flow

Run:

```bash
source /home/nvidia/twork/tinynav_setup.bash
cd /home/nvidia/twork/tinynav
bash scripts/tinynav_auto_nav.sh
```

By default this starts:

- RealSense if needed
- `perception_node.py`
- `planning_node.py`
- `map_node.py --tinynav_map_path output/latest_map`
- `cmd_vel_control.py`
- Go2 `/cmd_vel` bridge
- RViz goal-to-POI bridge
- static global occupancy publisher
- map keyframe diagnostics publisher
- RViz on TigerVNC display `:1`

Useful options:

```bash
bash scripts/tinynav_auto_nav.sh --no-go2
bash scripts/tinynav_auto_nav.sh --map /path/to/map
TINYNAV_RVIZ_CONFIG=/tinynav/docs/vis.rviz bash scripts/tinynav_auto_nav.sh
```

Default RViz config:

```text
/tinynav/docs/vis_with_global_map.rviz
```

This config includes global map diagnostics that the upstream baseline RViz file
does not show by default.

## RViz Goal Handling

TinyNav does not use Nav2 action goals. The current interaction path is:

```text
RViz 2D Goal Pose
  -> /goal_pose
  -> tool/rviz_goal_to_poi.py
  -> map/pois.json
  -> /mapping/cmd_pois
  -> map_node.py
```

Each RViz goal replaces the active POI list with one target unless the bridge is
started with `--append`.

## How To Judge Localization

Do not judge localization from the blocky occupancy grid alone. It is a low
resolution 2D projection of TinyNav's 3D occupancy state and is mainly useful
for planning context.

Use these RViz layers instead:

- `Map Keyframe Path`: saved map keyframe trajectory from `poses.npy`.
- `Map Keyframes`: yellow keyframe line and sampled arrows.
- `Current Pose In Map`: current estimated pose in the saved map frame.
- `Relocalization Successes`: successful `/map/relocalization` poses.
- `Global Plan`: `/mapping/global_plan`.
- `Local Trajectory`: `/planning/trajectory_path`.

Healthy localization looks like:

- current pose stays near the saved keyframe path when operating in mapped areas;
- current pose moves smoothly without large jumps;
- successful relocalization messages appear after restart or after moving to a mapped place;
- global plan and local trajectory do not cut through obvious obstacles.

Useful checks:

```bash
ros2 topic echo /mapping/current_pose_in_map
ros2 topic echo /map/relocalization
ros2 topic echo /mapping/nav_progress
ros2 topic echo /planning/trajectory_path
tmux capture-pane -pt tinynav_nav_auto:map -S -300 | rg "relocaliz|nav done|not enough|PnP"
```

Goal success is not a Nav2 action result. TinyNav publishes a transient
`/mapping/nav_done` boolean and logs:

```text
All POIs have been visited, nav done
```

Listen before a run if the one-shot result matters:

```bash
ros2 topic echo /mapping/nav_done
```

## Unitree Go2 Bridge

The Go2 control path intentionally uses the previous project's README bridge,
not TinyNav's built-in Unitree platform code:

```text
TinyNav /cmd_vel
  -> go2_cmd_bridge.py
  -> unitree_sdk2py.go2.sport.SportClient.Move(vx, vy, vyaw)
```

Network setup:

```bash
sudo ip addr add 192.168.123.100/24 dev eth0 2>/dev/null || true
sudo ip link set eth0 up
ping -I eth0 -c 1 192.168.123.161
```

Manual bridge:

```bash
source /home/nvidia/twork/tinynav_setup.bash
cd /home/nvidia/twork/tinynav
export UNITREE_NET_IF=eth0
export GO2_CMD_TOPIC=/cmd_vel
export GO2_MAX_VX=0.45
export GO2_MAX_VY=0.25
export GO2_MAX_WZ=0.80
export GO2_MIN_CMD_V=0.10
export GO2_MIN_CMD_W=0.20
bash scripts/run_go2_cmd_bridge.sh
```

Manual teleop through the same bridge:

```bash
bash scripts/run_keyboard_teleop.sh --topic /cmd_vel --linear-speed 0.30 --angular-speed 0.55
```

Navigation mode uses more conservative defaults:

- `GO2_MAX_VX=0.30`
- `GO2_MAX_VY=0.00`
- `GO2_MAX_WZ=0.70`
- `GO2_MIN_CMD_V=0.10`
- `GO2_MIN_CMD_W=0.20`

## VNC / RViz

The real GNOME desktop on `DISPLAY=:0` is black when HDMI is disconnected,
because Xorg starts with no active display mode. For reliable headless RViz use
TigerVNC:

```bash
export TINYNAV_VNC_PASSWORD='<private-vnc-password>'
bash /home/nvidia/twork/tinynav/scripts/run_visible_vnc.sh
```

Connect to:

```text
<jetson-lan-ip>:5901
password: value of TINYNAV_VNC_PASSWORD
```

RViz runs on:

```bash
export DISPLAY=:1
export XAUTHORITY=/home/nvidia/.Xauthority
```

If openbox places RViz partly offscreen:

```bash
DISPLAY=:1 XAUTHORITY=/home/nvidia/.Xauthority wmctrl -r RViz -e 0,0,0,1280,720
```

## Current Known Limits

- Map quality still depends heavily on recording motion and visual texture.
- Repeated `not enough landmarks` or `no valid PnP relocalization candidate found`
  means global visual relocalization is weak.
- The static occupancy layer is blocky by design; use keyframe trajectory and
  current pose overlays for localization evaluation.
- Runtime maps, bags, and temp DBs are intentionally not versioned.

## Minimal Daily Commands

```bash
source /home/nvidia/twork/tinynav_setup.bash
cd /home/nvidia/twork/tinynav

# Build a fresh map.
bash scripts/tinynav_auto_map.sh

# Navigate with Go2 bridge.
bash scripts/tinynav_auto_nav.sh

# Navigate without commanding Go2.
bash scripts/tinynav_auto_nav.sh --no-go2

# Attach logs.
tmux attach -t tinynav_nav_auto

# Stop.
tmux kill-session -t tinynav_nav_auto
```
