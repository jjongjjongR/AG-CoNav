#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
AGCONAV_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
GO2_DIR="${AGCONAV_ROOT}/src/unitree_go2_ros2_jazzy"
GO2_PATCH="${AGCONAV_ROOT}/patches/unitree_go2_ros2_jazzy.patch"

if [[ ! -r /etc/os-release ]]; then
  echo "[ERROR] /etc/os-release를 읽을 수 없습니다." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "[ERROR] 지원 환경은 Ubuntu 24.04입니다. 현재: ${PRETTY_NAME:-unknown}" >&2
  exit 1
fi

echo "[1/8] ROS 2 apt 저장소 및 시스템 의존성 설치"
sudo apt-get update
sudo apt-get install -y software-properties-common curl ca-certificates
sudo add-apt-repository -y universe
sudo apt-get update

if ! apt-cache show ros-jazzy-desktop >/dev/null 2>&1; then
  echo "      ROS 2 apt 저장소 등록"
  ROS_APT_SOURCE_VERSION="$(
    curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | sed -n 's/.*"tag_name":[[:space:]]*"\([^"]*\)".*/\1/p' \
      | head -n 1
  )"
  if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
    echo "[ERROR] ros-apt-source 최신 버전을 확인하지 못했습니다." >&2
    exit 1
  fi
  ROS_APT_DEB="$(mktemp --suffix=.deb /tmp/ros2-apt-source.XXXXXX)"
  curl -fL \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    -o "${ROS_APT_DEB}"
  sudo dpkg -i "${ROS_APT_DEB}"
  rm -f "${ROS_APT_DEB}"
fi

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-clearpath-simulator \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-pointcloud-to-laserscan \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep \
  python3-numpy \
  python3-scipy \
  python3-matplotlib \
  python3-opencv \
  mesa-utils

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "[ERROR] /opt/ros/jazzy/setup.bash가 없습니다. ROS 2 Jazzy 설치를 확인하세요." >&2
  exit 1
fi
# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "[2/8] Go2/CHAMP 고정 커밋 가져오기"
if [[ ! -d "${GO2_DIR}/.git" ]]; then
  vcs import "${AGCONAV_ROOT}/src" < "${AGCONAV_ROOT}/deps.repos"
fi

EXPECTED_GO2_COMMIT="$(awk '/^[[:space:]]*version:/ {print $2; exit}' "${AGCONAV_ROOT}/deps.repos")"
ACTUAL_GO2_COMMIT="$(git -C "${GO2_DIR}" rev-parse HEAD)"
if [[ -z "${EXPECTED_GO2_COMMIT}" || "${ACTUAL_GO2_COMMIT}" != "${EXPECTED_GO2_COMMIT}" ]]; then
  echo "[ERROR] Go2 커밋이 deps.repos와 다릅니다." >&2
  echo "        expected=${EXPECTED_GO2_COMMIT}" >&2
  echo "        actual=${ACTUAL_GO2_COMMIT}" >&2
  echo "        기존 작업을 보존하기 위해 자동 checkout하지 않습니다." >&2
  exit 1
fi

echo "[3/8] AG-CoNav Go2 패치 적용"
if git -C "${GO2_DIR}" apply --reverse --check "${GO2_PATCH}" >/dev/null 2>&1; then
  echo "      패치가 이미 적용되어 있습니다."
elif git -C "${GO2_DIR}" apply --check "${GO2_PATCH}"; then
  git -C "${GO2_DIR}" apply "${GO2_PATCH}"
else
  echo "[ERROR] Go2 패치를 적용할 수 없습니다. 외부 저장소 변경 상태를 확인하세요." >&2
  exit 1
fi

echo "[4/8] rosdep 의존성 설치"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "${AGCONAV_ROOT}/src" --ignore-src -r -y --rosdistro jazzy

echo "[5/8] Gazebo Fuel 월드·드론 리소스 내려받기"
for fuel_model in "House 1" "Gas Station" "Oak tree" "Pine Tree" "Lamp Post" "X3 UAV"; do
  gz fuel download \
    -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/${fuel_model}"

  # gz fuel은 통신 실패 때도 성공(0)을 반환하는 버전이 있으므로 캐시를 직접 확인한다.
  fuel_cache_dir="${HOME}/.gz/fuel/fuel.gazebosim.org/openrobotics/models/${fuel_model,,}"
  if ! compgen -G "${fuel_cache_dir}/*/model.sdf" >/dev/null; then
    echo "[ERROR] Gazebo Fuel 모델 다운로드를 확인할 수 없습니다: ${fuel_model}" >&2
    echo "        인터넷/DNS 연결을 확인한 뒤 설정 스크립트를 다시 실행하세요." >&2
    exit 1
  fi
done

echo "[6/8] 전체 workspace 빌드"
cd "${AGCONAV_ROOT}"
colcon build --symlink-install
# shellcheck disable=SC1091
set +u
source "${AGCONAV_ROOT}/install/setup.bash"
set -u

echo "[7/8] 설치 결과 검증"
for pkg in agconav_bringup agconav_description agconav_gz_bridge agconav_worlds \
           clearpath_gz unitree_go2_description unitree_go2_sim; do
  ros2 pkg prefix "${pkg}" >/dev/null
done

BRINGUP_PREFIX="$(ros2 pkg prefix agconav_bringup)"
CLEARPATH_YAML="${BRINGUP_PREFIX}/share/agconav_bringup/config/clearpath_a300/robot.yaml"
CONTROLLER_YAML="${BRINGUP_PREFIX}/share/agconav_bringup/config/leg_controllers.yaml"
if [[ ! -f "${CLEARPATH_YAML}" || ! -f "${CONTROLLER_YAML}" ]]; then
  echo "[ERROR] bringup 설정 설치가 불완전합니다." >&2
  echo "        ${CLEARPATH_YAML}" >&2
  echo "        ${CONTROLLER_YAML}" >&2
  exit 1
fi

python3 - "${CLEARPATH_YAML}" <<'PY'
import sys
from clearpath_config.clearpath_config import ClearpathConfig

config = ClearpathConfig(sys.argv[1])
if config.system.namespace != 'wheel':
    raise SystemExit(f"unexpected Clearpath namespace: {config.system.namespace!r}")
print(f"      Clearpath robot.yaml OK: namespace={config.system.namespace}")
PY

grep -q '<param name="initial_value">1.0143535</param>' \
  "${GO2_DIR}/unitree_go2_description/urdf/leg.xacro"
grep -q '<parameters>$(arg ros_control_file)</parameters>' \
  "${GO2_DIR}/unitree_go2_description/urdf/unitree_go2_gazebo.xacro"

ROS_LOG_DIR="${AGCONAV_ROOT}/log/setup-check" \
  ros2 launch agconav_bringup agconav_sim.launch.py --show-args >/dev/null

if [[ -n "${DISPLAY:-}" ]] && ! glxinfo -B >/dev/null 2>&1; then
  echo "[ERROR] OpenGL GUI 검증에 실패했습니다." >&2
  echo "        GPU 드라이버 설치 상태를 확인하고 재부팅한 뒤 다시 실행하세요." >&2
  exit 1
fi

echo "[8/8] 완료"
echo
echo "설정과 빌드 검증이 끝났습니다. 다음 명령으로 실행하세요."
echo "  cd ${AGCONAV_ROOT}"
echo "  ./scripts/run_simulation.sh"
