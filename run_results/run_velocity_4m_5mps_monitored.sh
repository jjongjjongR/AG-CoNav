#!/usr/bin/env bash
# 5m AGL velocity 비행(방법B 실험용, 4m 간격 5m/s) 실행 + 감시.
# 84m 방법B 실험(run_84m_velocity_monitored.sh)과 동일하게 module_a/f는 라이브로
# 안 돌린다 -- 방법B는 bag의 원본 /drone/points+/tf만 있으면 오프라인 처리
# 가능하고, attempt3에서 진단된 TF동결 유력원인(module_a까지 같이 도는 상태의
# 자원경합)을 애초에 피한다.
#
# attempt3/4에서 발견된 버그 수정 반영:
#  1) timeout -k 5 3 (SIGTERM 무시 대비 강제종료).
#  2) 마지막 웨이포인트(wp N/N) 도달 후에는 진행률 정지를 STALL로 오판하지
#     않는다 (SUMMARY.md "알려진 버그" 참조 -- 정상 완주를 오판해 중단시켰던 것).
#  3) module_a를 안 돌리므로 "TF lookup failed" 로그 신호가 없다 -- 대신
#     `ros2 topic hz /tf`로 주기마다 직접 /tf 발행 여부를 확인해 TF 동결을
#     독립적으로 감지한다.
#
# 사용: run_velocity_4m_5mps_monitored.sh <attempt_tag>
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

TAG="${1:-attempt1}"
FLIGHT_LOG="run_results/logs/velocity_4m_5mps_${TAG}.log"
STATUS_LOG="run_results/logs/status_log_4m_5mps_${TAG}.log"
BAG_DIR="bags/velocity_4m_5mps_${TAG}"
TIMEOUT=3600      # 60분 (등속 기준 569.5s의 ~6.3배 여유 -- attempt4 선례의 5.7배 여유율과 비슷한 수준)
DISK_LIMIT=80     # %
STALL_CHECKS=5    # 진행률 5회(=5분) 연속 불변이면 정지로 판단 (단, 마지막 wp 도달 후는 예외)
TF_HZ_STALL_CHECKS=3  # /tf가 3회(=3분) 연속 0Hz면 TF 동결로 판단

mkdir -p run_results/logs
: > "$STATUS_LOG"
rm -rf "$BAG_DIR"
# 이전 실행의 stale <TAG>_result.txt가 남아있으면 감시 루프가 시작하자마자
# "이미 완료됨"으로 오판한다(이번 세션에 실제로 겪은 버그) -- BAG_DIR과
# 동일하게 매 실행 시작 시 지운다.
rm -f "run_results/logs/velocity_4m_5mps_${TAG}_result.txt"

ros2 launch agconav_test_worlds experiment.launch.py \
  flight:=velocity \
  path_file:=path_100x100_5m_4m_5mps.yaml \
  cruise_speed_mps:=5.0 \
  module_a:=false \
  module_f:=false \
  bag_output:="$(pwd)/$BAG_DIR" \
  > "$FLIGHT_LOG" 2>&1 &
LPID=$!
echo "[$(date '+%F %T')] launch pid=$LPID tag=$TAG" | tee -a "$STATUS_LOG"

shutdown_launch() {
  local reason="$1"
  echo "[$(date '+%F %T')] SHUTDOWN 시작 — 사유: $reason" | tee -a "$STATUS_LOG"
  kill -INT "$LPID" 2>/dev/null
  # ros2 bag record는 launch 파일에서 sigterm_timeout=60/sigkill_timeout=30로
  # 최대 90초의 graceful shutdown 유예를 받는다(대용량 bag의 metadata.yaml
  # flush 시간 확보 -- experiment.launch.py 참고). 여기서 그보다 먼저
  # $LPID에 SIGTERM(=두 번째 시그널)을 보내면 launch가 즉시 더 거친 종료로
  # 넘어가 그 유예를 무력화하므로, 최소 100초는 기다린 뒤에만 에스컬레이션한다.
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

# 실측으로 발견된 버그: 이 조건(4m/5mps, bag ~20GB+)에서는 SIGINT 후 100초를
# 기다려도 ros2 bag record가 metadata.yaml을 다 못 쓰고 SIGTERM 에스컬레이션에
# 끊기는 경우가 실제로 있었다(정상 완주 COMPLETED인데도 bag이 깨짐). 디스크
# 속도가 느린 환경에서는 더 흔할 수 있으므로, 타임아웃을 늘리는 대신 종료 직후
# metadata.yaml 존재를 확인하고 없으면 `ros2 bag reindex`로 자동 복구한다
# (mcap 파일 자체는 청크 단위로 안전하게 쓰이므로 reindex로 복구 가능).
ensure_bag_metadata() {
  if [ -d "$BAG_DIR" ] && [ ! -f "$BAG_DIR/metadata.yaml" ]; then
    echo "[$(date '+%F %T')] *** metadata.yaml 없음 — bag 녹화가 완료 직전에 끊긴 것으로 보임. ros2 bag reindex로 자동 복구 시도 ***" | tee -a "$STATUS_LOG"
    if ros2 bag reindex -s mcap "$BAG_DIR" >> "$STATUS_LOG" 2>&1 && [ -f "$BAG_DIR/metadata.yaml" ]; then
      echo "[$(date '+%F %T')] reindex 성공 — metadata.yaml 복구됨" | tee -a "$STATUS_LOG"
    else
      echo "[$(date '+%F %T')] *** reindex 실패 — bag이 손상됐을 수 있음, 수동 확인 필요 ***" | tee -a "$STATUS_LOG"
    fi
  fi
}

START=$(date +%s)
LAST_PROGRESS=""
STALL_COUNT=0
TF_STALL_COUNT=0
while true; do
  NOW=$(date +%s)
  ELAPSED=$((NOW-START))

  if ! kill -0 "$LPID" 2>/dev/null; then
    echo "[$(date '+%F %T')] ${ELAPSED}s LAUNCH_PROCESS_DIED (완주했거나 죽었거나 — flight log 확인 필요)" | tee -a "$STATUS_LOG"
    echo "LAUNCH_DIED" > "run_results/logs/velocity_4m_5mps_${TAG}_result.txt"
    break
  fi

  PROGRESS=$(grep -o 'waypoint [0-9]*/ *[0-9]* 도달\|wp [0-9]*/[0-9]*' "$FLIGHT_LOG" 2>/dev/null | tail -1)
  DISK_LINE=$(df -h ~ | tail -1)
  DISK_PCT=$(echo "$DISK_LINE" | awk '{print $5}' | tr -d '%')
  BAG_SIZE=$(du -sh "$BAG_DIR" 2>/dev/null | awk '{print $1}')

  # /tf가 실제로 발행되고 있는지 3초 창으로 직접 확인 (module_a를 안 돌리므로
  # "TF lookup failed" 로그가 안 나온다 -- 이게 attempt3의 진짜 위험신호였던
  # exp_drone_tf_bridge 정지를 독립적으로 잡는 유일한 수단).
  TF_HZ_OUT=$(timeout -k 5 4 ros2 topic hz /tf --window 20 2>/dev/null | grep -o 'average rate: [0-9.]*' | head -1)
  if [ -z "$TF_HZ_OUT" ]; then
    TF_STALL_COUNT=$((TF_STALL_COUNT+1))
    echo "[$(date '+%F %T')] *** /tf 무응답(${TF_STALL_COUNT}/${TF_HZ_STALL_CHECKS}) ***" | tee -a "$STATUS_LOG"
  else
    TF_STALL_COUNT=0
  fi

  echo "[$(date '+%F %T')] ${ELAPSED}s progress=${PROGRESS:-N/A} bag_size=${BAG_SIZE:-N/A} tf=${TF_HZ_OUT:-없음} disk=${DISK_LINE}" | tee -a "$STATUS_LOG"

  if [ "$TF_STALL_COUNT" -ge "$TF_HZ_STALL_CHECKS" ]; then
    echo "[$(date '+%F %T')] *** /tf가 ${TF_HZ_STALL_CHECKS}분 연속 무응답 — TF_FROZEN ***" | tee -a "$STATUS_LOG"
    shutdown_launch "TF 동결(${TF_HZ_STALL_CHECKS}분 연속 무응답)"
    ensure_bag_metadata
    echo "TF_FROZEN" > "run_results/logs/velocity_4m_5mps_${TAG}_result.txt"
    exit 7
  fi

  # 정지(stall) 판단 -- 단, 마지막 웨이포인트(N/N)에 이미 도달했다면 path_status만
  # 기다리는 정상적인 종료 처리 구간일 수 있으므로 STALL로 오판하지 않는다
  # (SUMMARY.md에 기록된 기존 버그의 재발 방지).
  IS_LAST_WP=0
  if [ -n "$PROGRESS" ]; then
    NUM=$(echo "$PROGRESS" | grep -o '[0-9]*' | sed -n '1p')
    DEN=$(echo "$PROGRESS" | grep -o '[0-9]*' | sed -n '2p')
    if [ -n "$NUM" ] && [ -n "$DEN" ] && [ "$NUM" = "$DEN" ]; then
      IS_LAST_WP=1
    fi
  fi
  if [ "$IS_LAST_WP" -eq 1 ]; then
    STALL_COUNT=0
    LAST_PROGRESS="$PROGRESS"
  elif [ -n "$PROGRESS" ]; then
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
    ensure_bag_metadata
    echo "STALLED" > "run_results/logs/velocity_4m_5mps_${TAG}_result.txt"
    exit 6
  fi

  if [ -n "$DISK_PCT" ] && [ "$DISK_PCT" -ge "$DISK_LIMIT" ]; then
    echo "[$(date '+%F %T')] *** 디스크 사용량 ${DISK_PCT}% >= ${DISK_LIMIT}% — 즉시 중단 ***" | tee -a "$STATUS_LOG"
    shutdown_launch "디스크 사용량 ${DISK_PCT}% 임계치 초과"
    ensure_bag_metadata
    echo "DISK_THRESHOLD_STOP" > "run_results/logs/velocity_4m_5mps_${TAG}_result.txt"
    exit 5
  fi

  if grep -qi "Traceback\|Segmentation fault\|core dumped" "$FLIGHT_LOG" 2>/dev/null; then
    echo "[$(date '+%F %T')] 로그에서 에러/크래시 시그니처 발견" | tee -a "$STATUS_LOG"
    shutdown_launch "flight log 에러 시그니처"
    ensure_bag_metadata
    echo "ERROR_IN_LOG" > "run_results/logs/velocity_4m_5mps_${TAG}_result.txt"
    exit 4
  fi

  PSTATUS=$(timeout -k 5 3 ros2 topic echo /drone/path_status --once 2>/dev/null)
  if echo "$PSTATUS" | grep -q "data: true"; then
    echo "[$(date '+%F %T')] path_status=true — 완주" | tee -a "$STATUS_LOG"
    shutdown_launch "정상 완주"
    ensure_bag_metadata
    echo "COMPLETED" > "run_results/logs/velocity_4m_5mps_${TAG}_result.txt"
    exit 0
  fi

  if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
    echo "[$(date '+%F %T')] TIMEOUT ${ELAPSED}s" | tee -a "$STATUS_LOG"
    shutdown_launch "타임아웃 ${TIMEOUT}s"
    ensure_bag_metadata
    echo "TIMEOUT" > "run_results/logs/velocity_4m_5mps_${TAG}_result.txt"
    exit 2
  fi

  sleep 60
done
