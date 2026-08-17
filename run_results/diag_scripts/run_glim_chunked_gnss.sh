#!/usr/bin/env bash
# Phase 4: run_glim_chunked_param.sh에 GPS->/gnss 컨버터 노드(navsat_to_gnss_pose.py)를
# 나란히 띄우는 버전. gnss_global 모듈이 활성화된 config로만 쓴다.
# 사용: run_glim_chunked_gnss.sh <config_dir> <n_chunks> <out_name>
set -e
cd /home/hyunwoo-chae/AG-CoNav-test_main
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source ~/glim_ext_ws/install/setup.bash

CONFIG_DIR="$1"
N_CHUNKS="$2"
OUT_NAME="$3"
BAG_DIR="bags/velocity_4m_5mps_gps_attempt2"
TMP_DIR="bags/_chunk_gnss"
OUT_DIR="run_results/glim_diag_dumps/${OUT_NAME}"
mkdir -p "$OUT_DIR"

rm -rf /tmp/dump "$TMP_DIR"
nohup ros2 run glim_ros glim_rosnode --ros-args -p config_path:="$CONFIG_DIR" \
  > "run_results/logs/${OUT_NAME}_glim.log" 2>&1 &
sleep 5
GLIM_PID=$(pgrep -x glim_rosnode | tail -1)
echo "[$OUT_NAME] glim pid=$GLIM_PID"
echo "$GLIM_PID" > /tmp/glim_chunked_pid.txt

nohup python3 run_results/diag_scripts/navsat_to_gnss_pose.py \
  > "run_results/logs/${OUT_NAME}_converter.log" 2>&1 &
CONV_PID=$!
echo "[$OUT_NAME] converter pid=$CONV_PID"

# 메모리 가드 (4600MB, 이전 세션과 동일 임계값)
nohup bash -c '
while true; do
  PID=$(cat /tmp/glim_chunked_pid.txt 2>/dev/null)
  if [ -z "$PID" ] || ! kill -0 "$PID" 2>/dev/null; then break; fi
  RSS_KB=$(ps -p "$PID" -o rss= 2>/dev/null | tr -d " ")
  if [ -n "$RSS_KB" ]; then
    RSS_MB=$((RSS_KB/1024))
    if [ "$RSS_MB" -ge 4600 ]; then
      echo "[$(date "+%T")] 메모리 위험(${RSS_MB}MB) -- GLIM SIGINT 발송" >> run_results/logs/'"${OUT_NAME}"'_memguard.log
      kill -INT "$PID"
      break
    fi
  fi
  sleep 5
done
' > /dev/null 2>&1 &
MEMGUARD_PID=$!

for i in $(seq 0 $((N_CHUNKS-1))); do
  if ! kill -0 "$GLIM_PID" 2>/dev/null; then
    echo "[$OUT_NAME] chunk $i 전에 GLIM이 이미 종료됨(메모리 가드 발동 추정) -- 중단"
    break
  fi
  SRC="$BAG_DIR/velocity_4m_5mps_gps_attempt2_${i}.mcap"
  python3 run_results/add_point_times_single.py "$SRC" "$TMP_DIR" > /dev/null
  echo "[$OUT_NAME] chunk $i 변환 완료, 재생 시작"
  ros2 bag play "$TMP_DIR" --clock --rate 1.0 > /dev/null 2>&1
  echo "[$OUT_NAME] chunk $i 재생 완료"
  rm -rf "$TMP_DIR"
done

kill "$MEMGUARD_PID" 2>/dev/null
kill -9 "$CONV_PID" 2>/dev/null
if kill -0 "$GLIM_PID" 2>/dev/null; then
  kill -INT "$GLIM_PID"
  for i in $(seq 1 20); do
    kill -0 "$GLIM_PID" 2>/dev/null || break
    sleep 5
  done
  kill -0 "$GLIM_PID" 2>/dev/null && kill -9 "$GLIM_PID" 2>/dev/null
fi

cp /tmp/dump/odom_imu.txt /tmp/dump/odom_lidar.txt /tmp/dump/traj_imu.txt /tmp/dump/traj_lidar.txt "$OUT_DIR"/ 2>/dev/null \
  && echo "[$OUT_NAME] 덤프 저장 완료 -> $OUT_DIR" \
  || echo "[$OUT_NAME] 덤프 실패 (파일 없음)"
