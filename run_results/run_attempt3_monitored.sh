#!/usr/bin/env bash
# 4단계-1 velocity 비행 attempt3 실행 + 감시(진행률/디스크 사용량, 60초 주기).
# 80% 디스크 사용량 도달 시 즉시 SIGINT->TERM 에스컬레이션으로 중단하고 종료.
cd /home/hyunwoo-chae/AG-CoNav-test_main
source /opt/ros/jazzy/setup.bash
source install/setup.bash

FLIGHT_LOG=run_results/logs/velocity_4m_attempt3.log
STATUS_LOG=run_results/logs/status_log_attempt3.log
BAG_DIR=bags/velocity_4m_attempt3
TIMEOUT=4500   # 75분 (기존 확정값 유지)
DISK_LIMIT=80  # %

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
  # gz sim 잔여 프로세스 정리 (기존 절차 재사용)
  pkill -INT -f "gz sim|gz-sim" 2>/dev/null
  sleep 3
  pkill -TERM -f "gz sim|gz-sim" 2>/dev/null
  sleep 2
  REMAIN=$(pgrep -af "ros2 launch|gz sim|gz-sim|parameter_bridge|ros2 bag record|drone_" 2>/dev/null)
  echo "[$(date '+%F %T')] 잔여 프로세스 확인: ${REMAIN:-없음}" | tee -a "$STATUS_LOG"
}

START=$(date +%s)
while true; do
  NOW=$(date +%s)
  ELAPSED=$((NOW-START))

  if ! kill -0 "$LPID" 2>/dev/null; then
    echo "[$(date '+%F %T')] ${ELAPSED}s LAUNCH_PROCESS_DIED (완주했거나 죽었거나 — flight log 확인 필요)" | tee -a "$STATUS_LOG"
    break
  fi

  PROGRESS=$(grep -o 'waypoint [0-9]*/ *[0-9]* 도달\|wp [0-9]*/[0-9]*' "$FLIGHT_LOG" 2>/dev/null | tail -1)
  DISK_LINE=$(df -h ~ | tail -1)
  DISK_PCT=$(echo "$DISK_LINE" | awk '{print $5}' | tr -d '%')
  BAG_SIZE=$(du -sh "$BAG_DIR" 2>/dev/null | awk '{print $1}')

  echo "[$(date '+%F %T')] ${ELAPSED}s progress=${PROGRESS:-N/A} bag_size=${BAG_SIZE:-N/A} disk=${DISK_LINE}" | tee -a "$STATUS_LOG"

  if [ -n "$DISK_PCT" ] && [ "$DISK_PCT" -ge "$DISK_LIMIT" ]; then
    echo "[$(date '+%F %T')] *** 디스크 사용량 ${DISK_PCT}% >= ${DISK_LIMIT}% — 즉시 중단 ***" | tee -a "$STATUS_LOG"
    shutdown_launch "디스크 사용량 ${DISK_PCT}% 임계치 초과"
    echo "DISK_THRESHOLD_STOP" > run_results/logs/attempt3_result.txt
    exit 5
  fi

  if grep -qi "Traceback\|Segmentation fault\|core dumped" "$FLIGHT_LOG" 2>/dev/null; then
    echo "[$(date '+%F %T')] 로그에서 에러/크래시 시그니처 발견" | tee -a "$STATUS_LOG"
    shutdown_launch "flight log 에러 시그니처"
    echo "ERROR_IN_LOG" > run_results/logs/attempt3_result.txt
    exit 4
  fi

  PSTATUS=$(timeout 3 ros2 topic echo /drone/path_status --once 2>/dev/null)
  if echo "$PSTATUS" | grep -q "data: true"; then
    echo "[$(date '+%F %T')] path_status=true — 완주" | tee -a "$STATUS_LOG"
    shutdown_launch "정상 완주"
    echo "COMPLETED" > run_results/logs/attempt3_result.txt
    exit 0
  fi

  if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
    echo "[$(date '+%F %T')] TIMEOUT ${ELAPSED}s" | tee -a "$STATUS_LOG"
    shutdown_launch "타임아웃 ${TIMEOUT}s"
    echo "TIMEOUT" > run_results/logs/attempt3_result.txt
    exit 2
  fi

  sleep 60
done

echo "루프 종료 — flight log 마지막 상태 확인 필요" | tee -a "$STATUS_LOG"
echo "UNKNOWN_EXIT" > run_results/logs/attempt3_result.txt
