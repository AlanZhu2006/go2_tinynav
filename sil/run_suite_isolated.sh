#!/usr/bin/env bash
# Overnight G2 suite with per-episode node isolation (teleports break VO otherwise).
set -uo pipefail
LOG=/tmp/claude-1000
ENVSET="source /opt/ros/humble/setup.bash && cd /sil && export PYTHONPATH=/sil:/tinynav:\$PYTHONPATH GO2_LINGBOTNAV_CONFIG=$LOG/sil_config.yaml"
N=$(python3 -c "import json;print(len(json.load(open('$LOG/sil_episodes_spec.json'))['episodes']))")
for i in $(seq 0 $((N-1))); do
  echo ">>> episode $i: restarting perception+map for clean VO"
  docker exec sil bash -c "pkill -f 'tinynav.core.perception_nod[e]'; pkill -f 'tinynav.core.map_nod[e]'; true"
  sleep 3
  docker exec -d sil bash -c "$ENVSET && python3 -m tinynav.core.perception_node > $LOG/sil_perception.log 2>&1"
  docker exec -d sil bash -c "$ENVSET && rm -rf $LOG/sil_navdb && mkdir -p $LOG/sil_navdb && python3 -m tinynav.core.map_node --tinynav_db_path $LOG/sil_navdb --tinynav_map_path /tmp/claude-1000/sim_gauntlet/map_sim8 > $LOG/sil_map.log 2>&1"
  sleep 10
  docker exec sil bash -c "$ENVSET && python3 sil/episode_suite.py $LOG/sil_episodes_spec.json $i" 2>&1 | grep -E "^ep|SUITE"
done
echo ISOLATED_SUITE_DONE
