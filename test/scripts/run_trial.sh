#!/bin/bash
# 실험 1 시험 1회를 격리해서 돌린다. Gazebo 를 새로 띄우고, 끝나면 정리한다.
#   run_trial.sh <world.sdf> <tag> <kind> <value> <controller> <policy> <out.json>
set -o pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT" || exit 1
W=$1; TAG=$2; KIND=$3; VAL=$4; CTL=$5; POL=$6; OUT=$7
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

# 서버만(-s). GUI 는 띄우지 않는다.
gz sim "$W" -r -s -v 1 > "$LOG" 2>&1 &
sleep 12

ros2 launch "$ROOT/test/scripts/leg_terrain.launch.py" \
  world:="$W" controller:="$CTL" policy:="$POL" >> "$LOG" 2>&1 &

# 컨트롤러가 active 될 때까지 기다린다(libtorch 로드가 오래 걸린다).
NAME=rl_quadruped_controller
[ "$CTL" = "guide" ] && NAME=unitree_guide_controller
ok=0
for _ in $(seq 1 30); do
  sleep 6
  if timeout 10 ros2 control list_controllers 2>/dev/null | grep -q "${NAME}.*active"; then ok=1; break; fi
done
if [ $ok -eq 0 ]; then
  echo "{\"tag\":\"$TAG\",\"verdict\":\"컨트롤러실패\",\"controller\":\"$CTL\",\"policy\":\"$POL\",\"kind\":\"$KIND\",\"value\":$VAL}" > "$OUT"
  cleanup; exit 0
fi

timeout 900 python3 "$ROOT/test/scripts/leg_trial.py" \
  --tag "$TAG" --kind "$KIND" --value "$VAL" \
  --controller "$CTL" --policy "$POL" > "$OUT" 2>>"$LOG"
[ -s "$OUT" ] || echo "{\"tag\":\"$TAG\",\"verdict\":\"측정실패\",\"controller\":\"$CTL\",\"policy\":\"$POL\",\"kind\":\"$KIND\",\"value\":$VAL}" > "$OUT"
cleanup
