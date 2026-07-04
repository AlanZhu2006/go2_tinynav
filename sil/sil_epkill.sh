#!/bin/bash
# runs INSIDE the container: verified kill of perception+map (quoting-proof, pid-based)
for pat in "tinynav.core.perception_node" "tinynav.core.map_node"; do
  for pid in $(pgrep -f "$pat"); do kill $pid 2>/dev/null; done
done
for k in $(seq 1 10); do
  n=$(pgrep -cf "tinynav.core.(perception|map)_node" || true)
  [ "${n:-0}" = "0" ] && break
  sleep 1
done
for pat in "tinynav.core.perception_node" "tinynav.core.map_node"; do
  for pid in $(pgrep -f "$pat"); do kill -9 $pid 2>/dev/null; done
done
n=$(pgrep -cf "tinynav.core.(perception|map)_node" || true)
: > /tmp/claude-1000/sil_map.log          # root-owned: truncate in-container (host user cannot);
: > /tmp/claude-1000/sil_perception.log   # stale content once satisfied log-greps spuriously
echo "epkill done, remaining=$n"
