#!/bin/bash
# 실험 1 시험 1회를 격리해서 돌린다. Gazebo 를 새로 띄우고 끝나면 정리한다.
#   run_bench.sh <world.sdf> <tag> <kind> <value> <controller> <policy> <speed> <out.json>
set -o pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT" || exit 1
W=$1; TAG=$2; KIND=$3; VAL=$4; CTL=$5; POL=$6; SPD=$7; OUT=$8
LOG="$ROOT/test/logs/${TAG}.log"
mkdir -p "$(dirname "$OUT")" "$ROOT/test/logs"

cleanup() { "$ROOT/test/scripts/cleanup.sh"; }
cleanup

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$HOME/libtorch/lib:$LD_LIBRARY_PATH
# !! gz 는 시스템 플러그인을 LD_LIBRARY_PATH 가 아니라 이 경로에서 찾는다 !!
# 빠지면 월드는 뜨는데 ros2_control 하드웨어가 안 붙어
#   [Err] Failed to load system plugin [gz_quadruped_hardware-system]
# 가 뜨고 컨트롤러가 영영 active 가 되지 않는다.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/install/gz_quadruped_hardware/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

# !! 스폰 직후 로봇은 힘이 빠진 채로 떨어진다 !!
# gz_quadruped_hardware 는 kp/kd 명령을 0 으로 고정해 두고 시작하고
# (gz_system.cpp: joint_kp_cmd = 0.0), RL 컨트롤러도 PASSIVE 에서는 kp=0 이다.
# 그래서 컨트롤러가 붙을 때까지(libtorch 로드 6~20초) 관절이 완전히 자유롭다.
# 실측: 스폰 z 0.400 -> 0.068 (배를 깔고 누움). 0.15 에서 떨어뜨려도 같다.
# 월드를 일시정지해 두는 방법은 못 쓴다 — 물리가 멈추면 controller_manager 의
# update 가 안 돌아 switch_controller 가 300초 타임아웃으로 죽는다(실측).
# 서버만(-s). GUI 는 띄우지 않는다 — 여기서 재는 것은 숫자뿐이다.
gz sim "$W" -r -s -v 1 > "$LOG" 2>&1 &
sleep 12

# 스폰 높이. 웅크린 자세(seed)에서 발이 지면에 닿는 높이여야 자유낙하가 없다.
# 컨트롤러(libtorch)가 뜨는 동안 로봇은 무제어라, 높이 떨어뜨리면 그대로
# 배를 깔고 눕고 그 뒤 FIXEDSTAND 가 못 일으킨다(실측 z 0.068 고정).
INIT_Z=${INIT_Z:-0.50}
ros2 launch "$ROOT/test/scripts/leg_bench.launch.py" \
  controller:="$CTL" policy:="$POL" world_init_z:="$INIT_Z" >> "$LOG" 2>&1 &

# !! 여기서 ros2 control list_controllers 로 폴링하지 말 것 !!
# 컨트롤러가 뜰 때까지 5초마다 폴링해 봤는데, 그 폴링이 매번 ROS 노드를
# 새로 띄우느라 기립 구간에 CPU 를 뺏는다. 기립 성공률이 눈에 띄게 떨어졌다.
# 컨트롤러가 안 뜨는 경우는 bench_trial.py 가 controller_ready 를 못 받아
# "컨트롤러실패" 로 보고하므로 여기서 따로 볼 필요가 없다.

timeout 600 python3 "$ROOT/test/scripts/bench_trial.py" \
  --tag "$TAG" --kind "$KIND" --value "$VAL" \
  --controller "$CTL" --policy "$POL" --speed "$SPD" > "$OUT" 2>>"$LOG"
[ -s "$OUT" ] || echo "{\"tag\":\"$TAG\",\"verdict\":\"측정실패\",\"controller\":\"$CTL\",\"policy\":\"$POL\",\"kind\":\"$KIND\",\"value\":$VAL,\"speed\":$SPD}" > "$OUT"
cleanup
