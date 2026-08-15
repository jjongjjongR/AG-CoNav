#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/hyunwoo-chae/AG-CoNav-test_main/install/setup.bash
LOG=/home/hyunwoo-chae/AG-CoNav-test_main/run_results/logs/probe_84m.log
for i in $(seq 1 40); do
  if grep -q "waypoint.*로드 완료" "$LOG" 2>/dev/null; then
    echo "path player ready after ${i}0s-ish loop $i"
    break
  fi
  sleep 3
done
sleep 15
echo "--- capturing 3 points messages ---"
timeout 20 ros2 topic echo /drone/points --once > /tmp/probe_pts_1.txt 2>&1
sleep 5
timeout 20 ros2 topic echo /drone/points --once > /tmp/probe_pts_2.txt 2>&1
echo DONE
