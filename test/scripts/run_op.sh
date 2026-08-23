#!/bin/bash
# 실험 1 시험 1회. **운용 스폰 런치를 무수정으로 그대로 쓴다.**
#   run_op.sh <world.sdf> <tag> <kind> <value> <controller> <policy> <speed> <out.json>
#   controller: rl | guide | champ
#
# 왜 이렇게 바꿨나: 앞서 운용 런치를 "본떠서" 다시 짠 하네스로는 기립이
# 안 됐다. 스폰 순서·이벤트 체인·중계 FSM 을 내가 재구현한 것 자체가
# 변수였다. 운용 스택은 Nav2 목표까지 실제로 걸어간 실적이 있으므로,
# 그 구성을 한 글자도 바꾸지 않고 **월드만** 시험용 경사/단차로 갈아끼운다.
#
# 내가 추가하는 것은 계측용 두 가지뿐이고, 둘 다 별도 프로세스라 운용
# 런치를 건드리지 않는다.
#   1. /clock 브리지. 정답 3D 포즈는 op_trial.py 가 `gz topic -e` 로
#      월드에서 직접 읽는다(ros_gz_bridge 는 엔티티 이름을 안 옮겨서 못 쓴다).
#   2. op_trial.py — /leg/cmd_vel 에 Twist 만 낸다. 기립 FSM 은 운용
#      cmd_vel_to_control_input 이 몰고, 준비 완료는 그 노드가 내는
#      /leg/controller_ready 로 안다.
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
# 빠지면 월드는 뜨는데 ros2_control 하드웨어가 안 붙어 컨트롤러가 영영
# active 가 되지 않는다.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/install/gz_quadruped_hardware/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

WORLD_NAME=$(grep -o '<world name="[^"]*"' "$W" | head -1 | sed 's/.*name="//;s/"//')

gz sim "$W" -r -s -v 1 > "$LOG" 2>&1 &
sleep 12

# 계측용 포즈·시계 브리지. 운용 런치의 GroupAction(/tf, /odom 리맵) 밖이라
# 서로 간섭하지 않는다.
ros2 run ros_gz_bridge parameter_bridge \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" >> "$LOG" 2>&1 &

# !! 운용 런치. 인자 외에는 아무것도 바꾸지 않는다 !!
# guide 판은 그 파일을 그대로 복사해 컨트롤러만 바꾼 것이다.
READY_ARGS=""
if [ "$CTL" = "champ" ]; then
  # CHAMP 기준선. 기존 운용 런치를 그대로 쓴다(무수정).
  # FSM 이 없어 controller_ready 를 안 내므로 기립을 높이로 판정한다.
  READY_ARGS="--ready-mode height"
  ros2 launch agconav_bringup go2_spawn.launch.py \
    robot_name:=leg use_sim_time:=true \
    world_init_z:="${INIT_Z:-0.5}" >> "$LOG" 2>&1 &
elif [ "$CTL" = "guide" ]; then
  # !! guide 는 설계 최대가 0.4 m/s 다 !!
  # StateTrotting 이 v_cmd = invNormalize(ly, -0.4, 0.4) 로 ly 를 [-1,1]
  # 조이스틱 축으로 읽는다. 1.0 m/s 를 그대로 주면 ly=2.5 가 되어 축 범위를
  # 2.5배 넘고, 로봇이 앞으로 기울기만 하고 발을 못 뗀다(실측 진출 0.00 m).
  # 그래서 명령을 그 한계로 자른다.
  GSPD=$(python3 -c "print(min(float('$SPD'), 0.4))")
  # 발 들어올림 높이. 실험에서 바꿔가며 재려고 환경변수로 뺐다.
  ros2 launch "$ROOT/test/scripts/leg_guide_spawn.launch.py" \
    robot_name:=leg use_sim_time:=true max_linear:="$GSPD" \
    gait_height:="${GAIT_H:-0.15}" \
    world_init_z:="${INIT_Z:-0.5}" >> "$LOG" 2>&1 &
else
  ros2 launch agconav_bringup go2_rl_spawn.launch.py \
    robot_name:=leg use_sim_time:=true \
    model_folder:="$POL" max_linear:="$SPD" \
    world_init_z:="${INIT_Z:-0.5}" >> "$LOG" 2>&1 &
fi

timeout 900 python3 "$ROOT/test/scripts/op_trial.py" \
  --tag "$TAG" --kind "$KIND" --value "$VAL" \
  --controller "$CTL" --policy "$POL" --speed "$SPD" \
  --world "$WORLD_NAME" $READY_ARGS ${STEER_ARGS:-} > "$OUT" 2>>"$LOG"
[ -s "$OUT" ] || echo "{\"tag\":\"$TAG\",\"verdict\":\"측정실패\",\"controller\":\"$CTL\",\"policy\":\"$POL\",\"kind\":\"$KIND\",\"value\":$VAL,\"speed\":$SPD}" > "$OUT"
cleanup
