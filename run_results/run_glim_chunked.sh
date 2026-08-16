#!/usr/bin/env bash
# GPS GLIM 실험 전용: mcap 청크(0~11)를 하나씩 t필드 변환 -> 재생 -> 삭제 순환.
# 디스크가 원본(23G)+t필드본 전체(~26G) 동시 보유를 못 해서(실측, PROGRESS.md
# 5단계) 청크 단위(~2.5G)로만 임시 변환한다. GLIM은 라이브 노드로 계속
# 띄워둔 채(SLAM 상태 유지) 청크마다 bag play로 먹인다.
set -e
cd /home/hyunwoo-chae/AG-CoNav-test_main
source /opt/ros/jazzy/setup.bash
source install/setup.bash

BAG_DIR="bags/velocity_4m_5mps_gps_attempt2"
TMP_DIR="bags/_chunk_t"
N_CHUNKS=12

for i in $(seq 0 $((N_CHUNKS-1))); do
  SRC="$BAG_DIR/velocity_4m_5mps_gps_attempt2_${i}.mcap"
  echo "[$(date '+%T')] 청크 $i/$((N_CHUNKS-1)) 변환 시작: $SRC"
  python3 run_results/add_point_times_single.py "$SRC" "$TMP_DIR"
  DISK_PCT=$(df -h ~ | tail -1 | awk '{print $5}' | tr -d '%')
  echo "[$(date '+%T')] 청크 $i 변환 완료, disk=${DISK_PCT}%"
  if [ "$DISK_PCT" -ge 85 ]; then
    echo "위험: 디스크 ${DISK_PCT}% -- 중단"
    exit 1
  fi
  echo "[$(date '+%T')] 청크 $i 재생 시작 (rate 1.0, glim_rosnode가 실시간 소비)"
  ros2 bag play "$TMP_DIR" --clock --rate 1.0
  echo "[$(date '+%T')] 청크 $i 재생 완료"
  rm -rf "$TMP_DIR"
done
echo "[$(date '+%T')] 전체 12청크 처리 완료"
