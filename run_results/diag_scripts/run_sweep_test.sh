#!/usr/bin/env bash
# Phase 2 파라미터 스윕: 짧은 bag 슬라이스로 glim_rosnode 1회 실행 -> 덤프 저장.
# 사용: run_sweep_test.sh <config_dir> <out_name> <slice_bag_dir>
set -e
cd /home/hyunwoo-chae/AG-CoNav-test_main
source /opt/ros/jazzy/setup.bash
source install/setup.bash

CONFIG_DIR="$1"
OUT_NAME="$2"
SLICE_BAG="$3"
OUT_DIR="run_results/glim_diag_dumps/sweep_${OUT_NAME}"
mkdir -p "$OUT_DIR"

rm -rf /tmp/dump
nohup ros2 run glim_ros glim_rosnode --ros-args -p config_path:="$CONFIG_DIR" \
  > "run_results/logs/sweep_${OUT_NAME}_glim.log" 2>&1 &
sleep 5
GLIM_PID=$(pgrep -f "glim_ros/glim_rosnode" | tail -1)
echo "[$OUT_NAME] glim pid=$GLIM_PID"

ros2 bag play "$SLICE_BAG" --clock --rate 1.0 > /dev/null 2>&1

kill -INT "$GLIM_PID"
for i in $(seq 1 15); do
  kill -0 "$GLIM_PID" 2>/dev/null || break
  sleep 3
done
kill -0 "$GLIM_PID" 2>/dev/null && kill -9 "$GLIM_PID" 2>/dev/null

cp /tmp/dump/odom_imu.txt /tmp/dump/odom_lidar.txt "$OUT_DIR"/ 2>/dev/null || echo "[$OUT_NAME] 덤프 실패 (파일 없음)"
echo "[$OUT_NAME] done"
