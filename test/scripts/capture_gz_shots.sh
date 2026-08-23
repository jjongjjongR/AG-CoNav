#!/usr/bin/env bash
# Gazebo 화면 캡처 하네스 — 3로봇 스폰 장면 같은 논문용 스크린샷을 받는다.
#
# 세 가지가 함정이라 스크립트로 굳혀 둔다.
#
# 1) GUI 를 처음부터 켜면 안 된다. 성동구 월드 렌더가 CPU 를 가져가면
#    ros2_control 스포너가 list_controllers / switch_controller 호출에서
#    180초 타임아웃으로 죽고, rl_quadruped_controller 가 안 올라와
#    **Go2 가 주저앉은 채로 찍힌다**(실측). 서버를 headless 로 먼저 안정시킨
#    뒤 `gz sim -g` 로 GUI 를 붙이면 컨트롤러가 정상으로 올라온다.
# 2) /gui/screenshot 의 StringMsg 는 **파일명이 아니라 디렉터리**다.
#    전체 경로를 주면 서비스가 true 를 돌려주면서 아무것도 저장하지 않는다.
# 3) 월드의 <gui> 를 쓰면 사이드 패널 때문에 렌더 영역이 781x952 로 좁다.
#    test/configs/shot_gui.config 를 --gui-config 로 물리면 패널이 빠져
#    1853x963 이 나온다. 월드 파일은 건드리지 않는다.
#
# 사용법:  test/scripts/capture_gz_shots.sh [출력디렉터리]
set -o pipefail
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
OUT=${1:-$WS/results/final_maps/gz_shots}
LOG=$WS/test/logs/gz_shots.log
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DISPLAY=${DISPLAY:-:1}
mkdir -p "$OUT" "$WS/test/logs"
cd "$WS"

echo "[shot] 서버 기동(headless) $(date -Is)" | tee "$LOG"
ros2 launch agconav_bringup agconav_all.launch.py \
    headless:=true rviz:=false use_nav2:=false \
    enable_drone:=false enable_mapping:=false enable_fusion:=false \
    enable_traversability:=false auto_goals:=false \
    spawn_ground_early:=true >>"$LOG" 2>&1 &
SRV=$!
trap 'kill -INT $SRV 2>/dev/null; sleep 6; kill -9 $SRV 2>/dev/null' EXIT

echo "[shot] Go2 기립 대기 (rl_quadruped_controller)"
for _ in $(seq 1 120); do
    grep -q 'FIXEDSTAND' "$LOG" && break
    sleep 5
done
grep -q 'FIXEDSTAND' "$LOG" || { echo "[shot] Go2 기립 실패 — $LOG 확인"; exit 1; }

echo "[shot] GUI 부착"
gz sim -g --force-version 8 --gui-config "$WS/test/configs/shot_gui.config" >>"$LOG" 2>&1 &
GUI=$!
for _ in $(seq 1 60); do
    gz service -l 2>/dev/null | grep -q '/gui/screenshot' && break
    sleep 3
done
sleep 8

python3 "$WS/test/scripts/gz_camera_shots.py" "$OUT"
echo "[shot] 저장 위치: $OUT"
ls -la "$OUT"
kill -9 $GUI 2>/dev/null
