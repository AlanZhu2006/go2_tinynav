#!/usr/bin/env bash
# SIL bring-up: the FULL deployed ROS graph (real TRT, real Ceres) against Habitat.
#   host:      habitat bridge server (:5601, habitat env) + LingBot depth server (:5599, lingbot-map env)
#   container: bridge node + perception_node + map_node + planning_node + cmd_vel_control  (docker `sil`)
# Usage: bash sil/run_sil.sh [MAP_DIR] [SCENE_GLB]
set -uo pipefail
MAP="${1:-/tmp/claude-1000/sim_gauntlet/map_sim8}"
SCENE="${2:-/tmp/claude-1000/habitat_data/versioned_data/habitat_test_scenes/apartment_1.glb}"
SIL=/home/asus/Research/pengyue/go2_tinynav_mono_sim
GO2=/home/asus/Research/pengyue/go2_mono_nav
LOG=/tmp/claude-1000
CONDA=/home/asus/miniconda3/etc/profile.d/conda.sh

step() { echo; echo ">>> $*"; }

step "[1/5] host: habitat bridge server :5601"
pkill -f "habitat_bridge_serve[r]" 2>/dev/null
setsid nohup bash -c "source $CONDA && conda activate habitat && python $SIL/sil/habitat_bridge_server.py --scene '$SCENE' --port 5601" \
  > $LOG/sil_habitat.log 2>&1 < /dev/null &
until grep -q "listening" $LOG/sil_habitat.log 2>/dev/null; do sleep 2; done
echo "   up"

step "[2/5] host: LingBot depth server :5599 (map: $MAP)"
pkill -f "lingbot_depth_serve[r]" 2>/dev/null
setsid nohup bash -c "source $CONDA && conda activate lingbot-map && cd $GO2 && \
  ANCHORSCALE_PATH=/home/asus/Research/pengyue/AnchorScale LINGBOT_USE_SDPA=0 LINGBOT_MAP_DIR=$MAP \
  python -u perception_server/lingbot_depth_server.py --port 5599" \
  > $LOG/sil_depth.log 2>&1 < /dev/null &
until grep -q "listening" $LOG/sil_depth.log 2>/dev/null; do sleep 3; done
echo "   up"

step "[3/5] SIL config"
cat > $LOG/sil_config.yaml <<EOF
perception_server:
  host: 127.0.0.1
  port: 5599
  scale: 1.0
EOF

step "[4/5] container: ROS graph (docker exec, logs in $LOG/sil_*.log)"
docker exec sil bash -c "pkill -f 'tinynav.cor[e]|habitat_bridge_nod[e]|cmd_vel_contro[l]' 2>/dev/null; true"
ENVSET="source /opt/ros/humble/setup.bash && cd /sil && export PYTHONPATH=/sil:/tinynav:\$PYTHONPATH GO2_LINGBOTNAV_CONFIG=$LOG/sil_config.yaml"
docker exec -d sil bash -c "$ENVSET && python3 sil/habitat_bridge_node.py --rate 20 > $LOG/sil_bridge.log 2>&1"
sleep 3
docker exec -d sil bash -c "$ENVSET && python3 -m tinynav.core.perception_node > $LOG/sil_perception.log 2>&1"
docker exec -d sil bash -c "$ENVSET && mkdir -p $LOG/sil_navdb && python3 -m tinynav.core.map_node --tinynav_db_path $LOG/sil_navdb --tinynav_map_path $MAP > $LOG/sil_map.log 2>&1"
docker exec -d sil bash -c "$ENVSET && python3 -m tinynav.core.planning_node > $LOG/sil_planning.log 2>&1"
docker exec -d sil bash -c "$ENVSET && python3 -m tinynav.platforms.cmd_vel_control > $LOG/sil_control.log 2>&1"
echo "   5 processes launched"

step "[5/5] health (10s settle)"
sleep 10
docker exec sil bash -c "source /opt/ros/humble/setup.bash && timeout 6 ros2 topic hz /camera/camera/color/image_raw 2>/dev/null | head -1"
docker exec sil bash -c "source /opt/ros/humble/setup.bash && ros2 topic list | grep -E 'slam|mapping|planning|cmd_vel' | head -12"

cat <<'EOF'

SIL RUNNING. Next:
  goal:    publish /goal_pose or run rviz_goal_to_poi + goto_waypoint (inside container)
  scoring: /sim/gt_pose is ground truth; latency_probe.py works here too (L5 lower bound)
  logs:    /tmp/claude-1000/sil_{habitat,depth,bridge,perception,map,planning,control}.log
EOF
