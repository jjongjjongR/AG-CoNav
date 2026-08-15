#!/usr/bin/env bash
# Usage: wait_path_status.sh <timeout_sec> <launch_pid> <logfile>
TIMEOUT="$1"; LPID="$2"; LOG="$3"
source /opt/ros/jazzy/setup.bash
source /home/hyunwoo-chae/AG-CoNav-test_main/install/setup.bash
START=$(date +%s)
while true; do
  NOW=$(date +%s)
  ELAPSED=$((NOW-START))
  if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
    echo "TIMEOUT after ${ELAPSED}s"
    exit 2
  fi
  if ! kill -0 "$LPID" 2>/dev/null; then
    echo "LAUNCH_PROCESS_DIED after ${ELAPSED}s"
    exit 3
  fi
  OUT=$(timeout 3 ros2 topic echo /drone/path_status --once 2>/dev/null)
  if echo "$OUT" | grep -q "data: true"; then
    echo "PATH_STATUS_TRUE after ${ELAPSED}s"
    exit 0
  fi
  if grep -qi "Traceback\|Segmentation fault\|core dumped" "$LOG" 2>/dev/null; then
    echo "ERROR_IN_LOG after ${ELAPSED}s"
    exit 4
  fi
  sleep 5
done
