#!/usr/bin/env bash
# 4단계-1 velocity 비행 attempt4 실행 + 감시(진행률/디스크 사용량, 60초 주기).
# attempt3에서 발견된 두 가지 버그 수정:
#  1) `timeout 3 ros2 topic echo`가 SIGTERM 무시로 40분간 멈췄던 문제
#     -> `timeout -k 5 3`으로 5초 유예 후 강제종료(SIGKILL) 추가.
#  2) follower가 wp5239/5251에서 조용히 정지한 걸 못 잡던 문제
#     -> 진행률(wp 번호)이 5분(5회) 연속 안 바뀌면 STALLED로 판단해 중단.
cd /home/hyunwoo-chae/AG-CoNav-test_main
source /opt/ros/jazzy/setup.bash
source install/setup.bash

FLIGHT_LOG=run_results/logs/velocity_4m_attempt4.log
STATUS_LOG=run_results/logs/status_log_attempt4.log
BAG_DIR=bags/velocity_4m_attempt4
TIMEOUT=4500      # 75분 (기존 확정값 유지, 상한)
DISK_LIMIT=80     # %
STALL_CHECKS=5    # 진행률 5회(=5분) 연속 불변이면 정지로 판단

: > "$STATUS_LOG"

ros2 launch agconav_test_worlds experiment.launch.py \
  flight:=velocity \
  path_file:=path_100x100_5m_4m.yaml \
  cruise_speed_mps:=4.0 \
  module_a:=true \
  bag_output:="$(pwd)/$BAG_DIR" \
  > "$FLIGHT_LOG" 2>&1 &
LPID=$!
echo "launch pid=$LPID" | tee -a "$STATUS_LOG"

shutdown_launch() {
  local reason="$1"
  echo "[$(date '+%F %T')] SHUTDOWN 시작 — 사유: $reason" | tee -a "$STATUS_LOG"
  kill -INT "$LPID" 2>/dev/null
  for i in $(seq 1 15); do
    kill -0 "$LPID" 2>/dev/null || break
    sleep 2
  done
  if kill -0 "$LPID" 2>/dev/null; then
    echo "[$(date '+%F %T')] SIGINT로 안 죽어서 SIGTERM 에스컬레이션" | tee -a "$STATUS_LOG"
    kill -TERM "$LPID" 2>/dev/null
    sleep 5
  fi
  pkill -INT -f "gz sim|gz-sim" 2>/dev/null
  sleep 3
  pkill -TERM -f "gz sim|gz-sim" 2>/dev/null
  sleep 2
  # attempt3에서 SIGTERM에도 안 죽던 잔여 프로세스 대응 -> 마지막에 SIGKILL까지
  REMAIN_PIDS=$(pgrep -f "parameter_bridge|ros2 bag record|drone_elevation_mapper|velocity_path_follower.py|static_transform_publisher" 2>/dev/null)
  if [ -n "$REMAIN_PIDS" ]; then
    sleep 5
    for p in $REMAIN_PIDS; do
      kill -0 "$p" 2>/dev/null && kill -9 "$p" 2>/dev/null
    done
  fi
  REMAIN=$(pgrep -af "ros2 launch|gz sim|gz-sim|parameter_bridge|ros2 bag record|drone_" 2>/dev/null)
  echo "[$(date '+%F %T')] 잔여 프로세스 확인: ${REMAIN:-없음}" | tee -a "$STATUS_LOG"
}

START=$(date +%s)
LAST_PROGRESS=""
STALL_COUNT=0
while true; do
  NOW=$(date +%s)
  ELAPSED=$((NOW-START))

  if ! kill -0 "$LPID" 2>/dev/null; then
    echo "[$(date '+%F %T')] ${ELAPSED}s LAUNCH_PROCESS_DIED (완주했거나 죽었거나 — flight log 확인 필요)" | tee -a "$STATUS_LOG"
    echo "LAUNCH_DIED" > run_results/logs/attempt4_result.txt
    break
  fi

  PROGRESS=$(grep -o 'waypoint [0-9]*/ *[0-9]* 도달\|wp [0-9]*/[0-9]*' "$FLIGHT_LOG" 2>/dev/null | tail -1)
  DISK_LINE=$(df -h ~ | tail -1)
  DISK_PCT=$(echo "$DISK_LINE" | awk '{print $5}' | tr -d '%')
  BAG_SIZE=$(du -sh "$BAG_DIR" 2>/dev/null | awk '{print $1}')
  TF_FAILS=$(grep -c "TF lookup failed" "$FLIGHT_LOG" 2>/dev/null || echo 0)

  echo "[$(date '+%F %T')] ${ELAPSED}s progress=${PROGRESS:-N/A} bag_size=${BAG_SIZE:-N/A} tf_fail_count=${TF_FAILS} disk=${DISK_LINE}" | tee -a "$STATUS_LOG"

  # 정지(stall) 감지 — 진행률이 5분 연속 그대로면 follower/TF가 죽은 것으로 판단
  if [ -n "$PROGRESS" ]; then
    if [ "$PROGRESS" = "$LAST_PROGRESS" ]; then
      STALL_COUNT=$((STALL_COUNT+1))
    else
      STALL_COUNT=0
      LAST_PROGRESS="$PROGRESS"
    fi
  fi
  if [ "$STALL_COUNT" -ge "$STALL_CHECKS" ]; then
    echo "[$(date '+%F %T')] *** 진행률이 ${STALL_CHECKS}분 연속 정지(${PROGRESS}) — STALLED ***" | tee -a "$STATUS_LOG"
    shutdown_launch "진행률 정지(${STALL_CHECKS}분)"
    echo "STALLED" > run_results/logs/attempt4_result.txt
    exit 6
  fi

  if [ -n "$DISK_PCT" ] && [ "$DISK_PCT" -ge "$DISK_LIMIT" ]; then
    echo "[$(date '+%F %T')] *** 디스크 사용량 ${DISK_PCT}% >= ${DISK_LIMIT}% — 즉시 중단 ***" | tee -a "$STATUS_LOG"
    shutdown_launch "디스크 사용량 ${DISK_PCT}% 임계치 초과"
    echo "DISK_THRESHOLD_STOP" > run_results/logs/attempt4_result.txt
    exit 5
  fi

  if grep -qi "Traceback\|Segmentation fault\|core dumped" "$FLIGHT_LOG" 2>/dev/null; then
    echo "[$(date '+%F %T')] 로그에서 에러/크래시 시그니처 발견" | tee -a "$STATUS_LOG"
    shutdown_launch "flight log 에러 시그니처"
    echo "ERROR_IN_LOG" > run_results/logs/attempt4_result.txt
    exit 4
  fi

  PSTATUS=$(timeout -k 5 3 ros2 topic echo /drone/path_status --once 2>/dev/null)
  if echo "$PSTATUS" | grep -q "data: true"; then
    echo "[$(date '+%F %T')] path_status=true — 완주" | tee -a "$STATUS_LOG"
    shutdown_launch "정상 완주"
    echo "COMPLETED" > run_results/logs/attempt4_result.txt
    exit 0
  fi

  if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
    echo "[$(date '+%F %T')] TIMEOUT ${ELAPSED}s" | tee -a "$STATUS_LOG"
    shutdown_launch "타임아웃 ${TIMEOUT}s"
    echo "TIMEOUT" > run_results/logs/attempt4_result.txt
    exit 2
  fi

  sleep 60
done
