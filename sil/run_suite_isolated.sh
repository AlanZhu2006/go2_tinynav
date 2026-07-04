#!/usr/bin/env bash
# Overnight G2 suite with per-episode node isolation (teleports break VO otherwise).
set -uo pipefail
LOG=/tmp/claude-1000
ENVSET="source /opt/ros/humble/setup.bash && cd /sil && export PYTHONPATH=/sil:/tinynav:\$PYTHONPATH GO2_LINGBOTNAV_CONFIG=$LOG/sil_config.yaml"
SPEC="${SIL_SPEC:-$LOG/sil_episodes_spec.json}"
MAPD="${SIL_MAP:-/tmp/claude-1000/sim_gauntlet/map_sim8}"
N=$(python3 -c "import json;print(len(json.load(open('$SPEC'))['episodes']))")
for i in $(seq 0 $((N-1))); do
  echo ">>> episode $i: restarting perception+map for clean VO"
  # VERIFIED kill: leftover duplicates were the intermittent-starvation root cause (two
  # perceptions fighting over the single-client depth server + DDS).
  docker exec sil bash -c "
    pkill -f 'tinynav.core.perception_nod[e]'; pkill -f 'tinynav.core.map_nod[e]'
    for k in 1 2 3 4 5 6 7 8 9 10; do
      n=\$(pgrep -fc 'tinynav.core.(perception|map)_nod[e]' || true)
      [ \"\${n:-0}\" = \"0\" ] && break
      sleep 1
    done
    pkill -9 -f 'tinynav.core.perception_nod[e]' 2>/dev/null
    pkill -9 -f 'tinynav.core.map_nod[e]' 2>/dev/null
    true"
  sleep 2
  docker exec -d sil bash -c "$ENVSET && TINYNAV_CB_WATCHDOG=1 TINYNAV_IMG_QOS_RELIABLE=1 python3 -m tinynav.core.perception_node > $LOG/sil_perception.log 2>&1"
  docker exec -d sil bash -c "$ENVSET && rm -rf $LOG/sil_navdb && mkdir -p $LOG/sil_navdb && python3 -m tinynav.core.map_node --tinynav_db_path $LOG/sil_navdb --tinynav_map_path $MAPD > $LOG/sil_map.log 2>&1"
  sleep 10
  docker exec sil bash -c "$ENVSET && python3 sil/episode_suite.py $SPEC $i" 2>&1 | grep -E "^ep|SUITE"
done
echo ISOLATED_SUITE_DONE
