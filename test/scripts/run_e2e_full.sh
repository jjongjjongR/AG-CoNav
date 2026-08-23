#!/usr/bin/env bash
# A~F -> B -> C -> D -> E 단일 launch E2E 실행 + 완료 감지 자동 종료 + 검증.
# 파이프라인 자체에는 개입하지 않는다. 모듈 E 저장 신호가 뜨면 launch 를
# 정상 종료(SIGINT)시키고 verify_e2e_full.py 를 돌린다.
# set -u 금지 — ROS setup.bash 가 미설정 변수를 참조해 즉시 죽는다
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
LOG=$WS/test/logs/e2e_full.log
VER=$WS/test/logs/e2e_verify.txt
MAX_SEC=$((9*3600))       # 상한 9시간 (직전 검증 331분)

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd "$WS"

echo "[runner] start $(date -Is)" | tee "$LOG"
setsid ros2 launch agconav_bringup agconav_all.launch.py \
    use_nav2:=true headless:=true enable_drone:=true spawn_ground_early:=true \
    >>"$LOG" 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p $LPID | tr -d ' ')
echo "[runner] launch pid=$LPID pgid=$PGID" >> "$LOG"
echo "$LPID" > "$WS/test/logs/e2e_full.pid"

START=$(date +%s)
REASON=timeout
while true; do
    sleep 20
    if grep -q 'saved merged map to' "$LOG"; then REASON=complete; break; fi
    if ! kill -0 "$LPID" 2>/dev/null; then REASON=died; break; fi
    if [ $(( $(date +%s) - START )) -ge $MAX_SEC ]; then REASON=timeout; break; fi
done
echo "[runner] stop reason=$REASON $(date -Is)" >> "$LOG"

if [ "$REASON" != died ]; then
    sleep 20                                   # E 저장 flush 여유
    kill -INT -"$PGID" 2>/dev/null
    for _ in $(seq 1 60); do kill -0 "$LPID" 2>/dev/null || break; sleep 2; done
    kill -TERM -"$PGID" 2>/dev/null; sleep 5
    kill -KILL -"$PGID" 2>/dev/null
fi
pkill -f 'gz sim' 2>/dev/null; pkill -f 'ruby.*gz' 2>/dev/null
sleep 3

echo "[runner] verify $(date -Is)" >> "$LOG"
python3 "$WS/test/scripts/verify_e2e_full.py" "$LOG" > "$VER" 2>&1
echo "[runner] done reason=$REASON elapsed=$(( ($(date +%s)-START)/60 ))min" >> "$LOG"
exit 0
