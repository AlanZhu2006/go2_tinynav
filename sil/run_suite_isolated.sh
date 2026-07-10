#!/usr/bin/env bash
# Overnight G2 suite with per-episode node isolation (teleports break VO otherwise).
set -uo pipefail
LOG=/tmp/claude-1000
ENVSET="source /opt/ros/humble/setup.bash && cd /sil && export PYTHONPATH=/sil:/tinynav:\$PYTHONPATH GO2_LINGBOTNAV_CONFIG=$LOG/sil_config.yaml"
SPEC="${SIL_SPEC:-$LOG/sil_episodes_spec.json}"
MAPD="${SIL_MAP:-/tmp/claude-1000/sim_gauntlet/map_sim8}"
cp "$(dirname "$0")/sil_epkill.sh" $LOG/sil_epkill.sh
N=$(python3 -c "import json;print(len(json.load(open('$SPEC'))['episodes']))")
gpu_health() {
  docker exec sil nvidia-smi >/dev/null 2>&1
}

for i in $(seq 0 $((N-1))); do
  echo ">>> episode $i: restarting perception+map for clean VO"
  if ! gpu_health; then
    echo "SUITE_ABORT: container GPU/NVML health check failed before episode $i"
    exit 42
  fi
  # VERIFIED kill via staged script (inline quoting through docker exec proved fragile:
  # a zombie morning map_node survived every episode and duelled topics all afternoon)
  docker exec sil bash /tmp/claude-1000/sil_epkill.sh
  sleep 2
  if ! gpu_health; then
    echo "SUITE_ABORT: container GPU/NVML health check failed after epkill for episode $i"
    exit 42
  fi
  docker exec -d sil bash -c "$ENVSET && TINYNAV_CB_WATCHDOG=1 TINYNAV_IMG_QOS_RELIABLE=1 python3 -m tinynav.core.perception_node > $LOG/sil_perception.log 2>&1"
  docker exec -d sil bash -c "$ENVSET && rm -rf $LOG/sil_navdb && mkdir -p $LOG/sil_navdb && python3 -m tinynav.core.map_node --tinynav_db_path $LOG/sil_navdb --tinynav_map_path $MAPD > $LOG/sil_map.log 2>&1"
  sleep 10
  if ! gpu_health; then
    echo "SUITE_ABORT: container GPU/NVML health check failed after node start for episode $i"
    exit 42
  fi
  if ! docker exec sil bash -lc "pgrep -f 'tinynav.core.[p]erception_node' >/dev/null && pgrep -f 'tinynav.core.[m]ap_node' >/dev/null"; then
    echo "SUITE_ABORT: perception/map failed to stay up for episode $i"
    docker exec sil bash -lc "tail -n 40 $LOG/sil_perception.log; tail -n 40 $LOG/sil_map.log" 2>&1
    exit 43
  fi
  docker exec sil bash -c "$ENVSET && python3 sil/episode_suite.py $SPEC $i" 2>&1 | grep -E "^ep|SUITE"
done
echo ISOLATED_SUITE_DONE
