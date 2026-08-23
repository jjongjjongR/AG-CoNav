#!/bin/bash
# quadruped_ros2_control 을 vcs 로 받은 뒤 실행한다.
#
#   vcs import src < deps.repos
#   ./scripts/prune_quadruped_deps.sh
#   colcon build --symlink-install
#
# 왜 필요한가: 받은 그대로 빌드하면 **실패한다.**
#   - ocs2_quadruped_controller 가 coal 과 hpp-fcl 을 동시에 끌어와 충돌한다.
#   - hardware_unitree_sdk2 는 실물 Go2 전용이라 unitree_sdk2 가 있어야 빌드된다.
#     우리는 Gazebo 로만 돌리므로 필요 없다.
# 나머지 로봇(a1, aliengo, b2, go1, anybotics, deep_robotics 등)은 쓰지 않는데
# 메시까지 딸려 있어 용량과 빌드 시간만 먹는다.
#
# 지우지 않고 COLCON_IGNORE 를 두는 방식을 쓴다. 원본을 보존하므로 나중에
# 다른 로봇이 필요해지면 파일 하나만 지우면 되고, vcs 가 트리를 더럽혔다고
# 보지도 않는다.
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
Q="$ROOT/src/quadruped_ros2_control"
[ -d "$Q" ] || { echo "없음: $Q  (먼저 vcs import src < deps.repos)"; exit 1; }

IGNORE=(
  "controllers/ocs2_quadruped_controller"   # coal/hpp-fcl 충돌 — 빌드 실패
  "libraries/qpoases_colcon"                # 위 컨트롤러 전용
  "hardwares/hardware_unitree_sdk2"         # 실물 전용, unitree_sdk2 필요
  "commands/unitree_joystick_input"         # 실물 조이스틱 전용
  # 아래 넷은 빌드는 되지만 아무도 안 쓴다. 우리 코드에서 참조가 0 이고
  # 다른 패키지가 depend 하지도 않는다. 되살리려면 COLCON_IGNORE 만 지우면 된다.
  "libraries/gz_quadruped_playground"       # 이 저장소 자체 데모용 월드
  "commands/joystick_input"                 # 조이스틱 — 우리는 cmd_vel_to_control_input 을 쓴다
  "commands/keyboard_input"                 # 키보드 수동 조작 — 필요하면 되살릴 것
  "descriptions/anybotics"
  "descriptions/deep_robotics"
  "descriptions/magiclab&xiaomi"
  "descriptions/unitree/a1_description"
  "descriptions/unitree/aliengo_description"
  "descriptions/unitree/b2_description"
  "descriptions/unitree/go1_description"
)
for d in "${IGNORE[@]}"; do
  if [ -d "$Q/$d" ]; then
    touch "$Q/$d/COLCON_IGNORE"
    echo "  건너뜀: $d"
  fi
done

# !! unitree_guide_controller 를 건너뛰지 말 것 !!
# 예전에는 "컨트롤러 실험에서 탈락(docs/11)" 이라고 제외했는데, 그 실험은
# 관절 초기 자세 시딩이 깨진 상태에서 잰 것이라 근거를 잃었다. 다시 재니
# 램프에서 20도까지 등판해 RL(15도)보다 낫다.
#
# !! 다만 운용 기본은 rl_quadruped_controller 다 !!
# guide 는 램프에서는 잘 걷지만 Nav2 종단 주행에서 반복 전복해 운용에서
# 뺐다(docs/14). guide 는 비교·회귀 확인용으로 남겨 두는 것이므로 여기서
# 제외하면 안 된다.
#
# 우리가 쓰는 것: rl_quadruped_controller(운용 기본)
# + unitree_guide_controller(비교용) + go2_description + gz 하드웨어.
# 정책 가중치는 descriptions/unitree/go2_description/config/robot_lab/policy.pt 이고
# 런치에서 model_folder:=robot_lab 로 고른다(config_folder 아니다 — 자주 헷갈린다).
echo
echo "확인: $(ls "$Q/descriptions/unitree/go2_description/config/robot_lab/policy.pt" 2>/dev/null && echo 'RL 정책 있음' || echo '!! RL 정책 없음')"
echo "libtorch 도 있어야 rl_quadruped_controller 가 빌드된다:"
echo "  export CMAKE_PREFIX_PATH=\$HOME/libtorch:\$CMAKE_PREFIX_PATH"
