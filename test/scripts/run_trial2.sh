#!/bin/bash
# 실험 1 시험 1회. 운용 스폰 런치를 그대로 쓴다.
#   run_trial2.sh <world.sdf> <tag> <kind> <value> <controller> <policy> <speed> <out.json>
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
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/install/gz_quadruped_hardware/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

gz sim "$W" -r -s -v 1 > "$LOG" 2>&1 &
sleep 12

if [ "$CTL" = "guide" ]; then
  LAUNCH="$ROOT/test/scripts/leg_guide_spawn.launch.py"
  ARGS="world_init_z:=0.4"
else
  LAUNCH="$ROOT/install/agconav_bringup/share/agconav_bringup/launch/go2_rl_spawn.launch.py"
  ARGS="model_folder:=$POL max_linear:=$SPD world_init_z:=0.4"
fi
ros2 launch "$LAUNCH" robot_name:=leg use_sim_time:=true $ARGS >> "$LOG" 2>&1 &

timeout 900 python3 "$ROOT/test/scripts/leg_trial2.py" \
  --tag "$TAG" --kind "$KIND" --value "$VAL" \
  --controller "$CTL" --policy "$POL" --speed "$SPD" > "$OUT" 2>>"$LOG"
[ -s "$OUT" ] || echo "{\"tag\":\"$TAG\",\"verdict\":\"측정실패\",\"controller\":\"$CTL\",\"policy\":\"$POL\",\"kind\":\"$KIND\",\"value\":$VAL}" > "$OUT"
cleanup
