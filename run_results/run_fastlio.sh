#!/usr/bin/env bash
# FAST-LIO2(spark-fast-lio) 실행 스크립트. 완전히 별도 워크스페이스
# (~/fastlio_ws)의 노드를 띄우고, 우리 bag(AG-CoNav-test_main 소스는
# 안 건드림)을 재생해 궤적을 기록한다.
#
#   run_fastlio.sh <bag_dir> <태그> <start_offset_s> <duration_s|-1> [rate] [extra -p 파라미터...]
set -e
BAG="$1"; TAG="$2"; START_OFFSET="${3:-0}"; DURATION="${4:-1}"; RATE="${5:-1.0}"; shift 5 || true
EXTRA_PARAMS=("$@")
ROOT="/home/hyunwoo-chae/AG-CoNav-test_main"
FLROOT="/home/hyunwoo-chae/fastlio_ws"
RR="$ROOT/run_results"
OUT="/tmp/fastlio_$TAG"
DOMAIN=93

source /opt/ros/jazzy/setup.bash
source "$FLROOT/install/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN"

pkill -f "spark_lio_mapping"            2>/dev/null || true
pkill -f "fastlio_points_republisher"   2>/dev/null || true
pkill -f "fastlio_traj_recorder"        2>/dev/null || true
pkill -f "ros2 bag play"                2>/dev/null || true
sleep 3

rm -rf "$OUT"; mkdir -p "$OUT"
echo "[$(date '+%T')] tag=$TAG start_offset=$START_OFFSET duration=$DURATION rate=$RATE"

python3 "$RR/fastlio_points_republisher.py" /drone/points /drone/points_ouster \
  > "$OUT/republisher.log" 2>&1 &
REPUB_PID=$!

python3 "$RR/fastlio_traj_recorder.py" "$OUT/traj_lidar.txt" /odometry \
  > "$OUT/recorder.log" 2>&1 &
RECORDER_PID=$!

ros2 run spark_fast_lio spark_lio_mapping --ros-args \
  -r lidar:=/drone/points_ouster \
  -r imu:=/drone/imu \
  -p use_sim_time:=true \
  --params-file "$FLROOT/agconav_config.yaml" \
  "${EXTRA_PARAMS[@]}" \
  > "$OUT/fastlio.log" 2>&1 &
FASTLIO_PID=$!

sleep 5

DUR_ARGS=()
if [ "$DURATION" != "-1" ]; then
  DUR_ARGS=(--playback-duration "$DURATION")
fi

ros2 bag play "$BAG" --clock --rate "$RATE" --start-offset "$START_OFFSET" "${DUR_ARGS[@]}" \
  > "$OUT/play.log" 2>&1
echo "[$(date '+%T')] bag play 종료, 후처리 대기 중..."
sleep 5

kill -INT "$FASTLIO_PID" 2>/dev/null || true
sleep 3
kill "$REPUB_PID" "$RECORDER_PID" 2>/dev/null || true
sleep 1
kill -9 "$FASTLIO_PID" "$REPUB_PID" "$RECORDER_PID" 2>/dev/null || true

echo "[$(date '+%T')] 완료. 로그: $OUT  궤적: $OUT/traj_lidar.txt"
wc -l "$OUT/traj_lidar.txt" 2>/dev/null || echo "궤적 파일 없음(실패 가능성)"
