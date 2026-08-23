#!/bin/bash
# wheel(A300) 지형 주파 시험 1회.
#   run_wheel.sh <world.sdf> <tag> <kind> <value> <speed> <out.json>
#
# 문서 12 의 재확인이다. 그때 쓴 하네스는 리팩터링 때 삭제돼 새로 만들었다.
# leg 와 달리 clearpath 스택이 스폰을 맡으므로 그 런치를 그대로 쓴다.
set -o pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT" || exit 1
W=$1; TAG=$2; KIND=$3; VAL=$4; SPD=$5; OUT=$6
LOG="$ROOT/test/logs/${TAG}.log"
mkdir -p "$(dirname "$OUT")" "$ROOT/test/logs"

cleanup() { "$ROOT/test/scripts/cleanup.sh"; }
cleanup

source /opt/ros/jazzy/setup.bash
source install/setup.bash
WORLD_NAME=$(grep -o '<world name="[^"]*"' "$W" | head -1 | sed 's/.*name="//;s/"//')
# !! clearpath 모델은 gz_ros2_control 플러그인을 쓴다 !!
# 이 경로가 없으면 월드는 뜨는데
#   [Err] Failed to load system plugin [libgz_ros2_control-system.so]
# 로 하드웨어가 안 붙어 컨트롤러 스포너가 락 대기만 하다 죽는다.
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
SETUP="$ROOT/install/agconav_bringup/share/agconav_bringup/config/clearpath_a300"

gz sim "$W" -r -s -v 1 > "$LOG" 2>&1 &
sleep 12

# 계측용 브리지. 포즈는 PoseArray 로 받는다(이름은 안 오지만 순서는 보존된다).
ros2 run ros_gz_bridge parameter_bridge \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  "/world/${WORLD_NAME}/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V" \
  >> "$LOG" 2>&1 &

# clearpath 스폰. generate:=false 는 이미 만들어 둔 산출물을 쓴다는 뜻이다.
ros2 launch clearpath_gz robot_spawn.launch.py \
  setup_path:="$SETUP" world:="$WORLD_NAME" use_sim_time:=true \
  rviz:=false generate:=false x:=0.0 y:=0.0 z:=0.3 yaw:=0.0 >> "$LOG" 2>&1 &

# !! 포즈 첨자는 셸에서 미리 찾아 넘긴다 !!
# PoseArray 에는 엔티티 이름이 없다. 노드 안에서 찾으면 subprocess 가 스핀을
# 막아 신호를 놓친다(leg 에서 당했다). 여기서 한 번만 텍스트로 읽어 첨자를
# 구한 뒤 인자로 준다.
IDX=""
for _ in $(seq 1 30); do
  sleep 5
  NAMES=$(gz topic -e -t "/world/${WORLD_NAME}/pose/info" -n 1 2>/dev/null \
          | grep -oP '(?<=name: ")[^"]+')
  # clearpath 는 모델 이름을 네임스페이스로 짓는다(gz 토픽이 /model/wheel/robot/...).
  # a300 이라는 이름은 나오지 않는다. 지형 쪽 고정 이름을 빼고 남는 것을 고른다.
  IDX=$(echo "$NAMES" | awk '/robot|a300|A300|wheel/ {print NR-1; exit}')
  if [ -n "$IDX" ]; then
    echo "pose/info 엔티티: $(echo "$NAMES" | tr '\n' ' ')" >> "$LOG"
    break
  fi
done
if [ -z "$IDX" ]; then
  echo "{\"tag\":\"$TAG\",\"verdict\":\"스폰실패\",\"kind\":\"$KIND\",\"value\":$VAL,\"speed\":$SPD}" > "$OUT"
  cleanup; exit 0
fi
echo "a300 pose index = $IDX" >> "$LOG"

timeout 600 python3 "$ROOT/test/scripts/wheel_trial.py" \
  --tag "$TAG" --kind "$KIND" --value "$VAL" --speed "$SPD" \
  --world "$WORLD_NAME" --pose-index "$IDX" > "$OUT" 2>>"$LOG"
[ -s "$OUT" ] || echo "{\"tag\":\"$TAG\",\"verdict\":\"측정실패\",\"kind\":\"$KIND\",\"value\":$VAL,\"speed\":$SPD}" > "$OUT"
cleanup
