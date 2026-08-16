#!/usr/bin/env bash
# 84m 고정고도 velocity 비행(방법B 실험용) 실행 + 감시.
# path_100x100.yaml 그대로(84m/8m/s/10웨이포인트, RESULTS.md에서 안정성 확인된 조합).
# module_a/f는 끔 -- 방법B는 bag의 원본 /drone/points+/tf만 있으면 되고, 라이브로
# 지도/주행성을 돌릴 필요가 없다(5m AGL 실험에서 배운 대로 불필요한 I/O 부하 최소화).
cd /home/hyunwoo-chae/AG-CoNav-test_main
source /opt/ros/jazzy/setup.bash
source install/setup.bash

FLIGHT_LOG=run_results/logs/velocity_84m.log
STATUS_LOG=run_results/logs/status_log_84m.log
BAG_DIR=bags/velocity_84m
TIMEOUT=1200      # 20분 (84m 짧은 경로 기준, RESULTS.md 원래 가정과 일치)
DISK_LIMIT=80     # %
STALL_CHECKS=5    # 진행률 5회(=5분) 연속 불변이면 정지로 판단

: > "$STATUS_LOG"
# stale <result.txt>가 남아있으면 감시 루프가 시작하자마자 "이미 완료됨"으로
# 오판한다(5m/4m 실험에서 실제로 겪은 버그와 동일 클래스) -- 매 실행 시작 시 지운다.
rm -f run_results/logs/velocity_84m_result.txt

ros2 launch agconav_test_worlds experiment.launch.py \
  flight:=velocity \
  module_a:=false \
  module_f:=false \
  bag_output:="$(pwd)/$BAG_DIR" \
  > "$FLIGHT_LOG" 2>&1 &
LPID=$!
echo "launch pid=$LPID" | tee -a "$STATUS_LOG"

shutdown_launch() {
  local reason="$1"
  echo "[$(date '+%F %T')] SHUTDOWN 시작 — 사유: $reason" | tee -a "$STATUS_LOG"
  kill -INT "$LPID" 2>/dev/null
  # ros2 bag record는 launch 파일에서 sigterm_timeout=60/sigkill_timeout=30로
  # 최대 90초의 graceful shutdown 유예를 받는다(metadata.yaml flush 시간 확보 --
  # experiment.launch.py 참고, 5m/4m 실험에서 이 유예가 없어 metadata.yaml이
  # 누락됐던 버그를 고치며 추가함). 그보다 먼저 $LPID에 SIGTERM을 보내면 launch가
  # 즉시 더 거친 종료로 넘어가 그 유예를 무력화하므로, 최소 100초는 기다린다.
  for i in $(seq 1 50); do
    kill -0 "$LPID" 2>/dev/null || break
    sleep 2
  done
  if kill -0 "$LPID" 2>/dev/null; then
    echo "[$(date '+%F %T')] SIGINT로 100초 내 안 죽어서 SIGTERM 에스컬레이션" | tee -a "$STATUS_LOG"
    kill -TERM "$LPID" 2>/dev/null
    sleep 5
  fi
  pkill -INT -f "gz sim|gz-sim" 2>/dev/null
  sleep 3
  pkill -TERM -f "gz sim|gz-sim" 2>/dev/null
  sleep 2
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
    echo "LAUNCH_DIED" > run_results/logs/velocity_84m_result.txt
    break
  fi

  PROGRESS=$(grep -o 'waypoint [0-9]*/ *[0-9]* 도달\|wp [0-9]*/[0-9]*' "$FLIGHT_LOG" 2>/dev/null | tail -1)
  DISK_LINE=$(df -h ~ | tail -1)
  DISK_PCT=$(echo "$DISK_LINE" | awk '{print $5}' | tr -d '%')
  BAG_SIZE=$(du -sh "$BAG_DIR" 2>/dev/null | awk '{print $1}')

  echo "[$(date '+%F %T')] ${ELAPSED}s progress=${PROGRESS:-N/A} bag_size=${BAG_SIZE:-N/A} disk=${DISK_LINE}" | tee -a "$STATUS_LOG"

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
    echo "STALLED" > run_results/logs/velocity_84m_result.txt
    exit 6
  fi

  if [ -n "$DISK_PCT" ] && [ "$DISK_PCT" -ge "$DISK_LIMIT" ]; then
    echo "[$(date '+%F %T')] *** 디스크 사용량 ${DISK_PCT}% >= ${DISK_LIMIT}% — 즉시 중단 ***" | tee -a "$STATUS_LOG"
    shutdown_launch "디스크 사용량 ${DISK_PCT}% 임계치 초과"
    echo "DISK_THRESHOLD_STOP" > run_results/logs/velocity_84m_result.txt
    exit 5
  fi

  if grep -qi "Traceback\|Segmentation fault\|core dumped" "$FLIGHT_LOG" 2>/dev/null; then
    echo "[$(date '+%F %T')] 로그에서 에러/크래시 시그니처 발견" | tee -a "$STATUS_LOG"
    shutdown_launch "flight log 에러 시그니처"
    echo "ERROR_IN_LOG" > run_results/logs/velocity_84m_result.txt
    exit 4
  fi

  PSTATUS=$(timeout -k 5 3 ros2 topic echo /drone/path_status --once 2>/dev/null)
  if echo "$PSTATUS" | grep -q "data: true"; then
    echo "[$(date '+%F %T')] path_status=true — 완주" | tee -a "$STATUS_LOG"
    shutdown_launch "정상 완주"
    echo "COMPLETED" > run_results/logs/velocity_84m_result.txt
    exit 0
  fi

  if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
    echo "[$(date '+%F %T')] TIMEOUT ${ELAPSED}s" | tee -a "$STATUS_LOG"
    shutdown_launch "타임아웃 ${TIMEOUT}s"
    echo "TIMEOUT" > run_results/logs/velocity_84m_result.txt
    exit 2
  fi

  sleep 60
done
