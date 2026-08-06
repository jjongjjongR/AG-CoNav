#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
AGCONAV_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "[ERROR] ROS 2 Jazzy가 없습니다. 먼저 ./scripts/setup_simulation.sh를 실행하세요." >&2
  exit 1
fi
if [[ ! -f "${AGCONAV_ROOT}/install/setup.bash" ]]; then
  echo "[ERROR] workspace가 빌드되지 않았습니다. 먼저 ./scripts/setup_simulation.sh를 실행하세요." >&2
  exit 1
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
source "${AGCONAV_ROOT}/install/setup.bash"
set -u

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="${ROS_LOG_DIR:-${AGCONAV_ROOT}/log/runtime}"
mkdir -p "${ROS_LOG_DIR}"

for pkg in agconav_bringup agconav_description agconav_gz_bridge agconav_worlds \
           clearpath_gz unitree_go2_description unitree_go2_sim; do
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "[ERROR] 필수 ROS 패키지를 찾지 못했습니다: ${pkg}" >&2
    echo "        ./scripts/setup_simulation.sh를 다시 실행하세요." >&2
    exit 1
  fi
done

BRINGUP_PREFIX="$(ros2 pkg prefix agconav_bringup)"
CLEARPATH_YAML="${BRINGUP_PREFIX}/share/agconav_bringup/config/clearpath_a300/robot.yaml"
if [[ ! -f "${CLEARPATH_YAML}" ]]; then
  echo "[ERROR] 설치된 Clearpath 설정이 없습니다: ${CLEARPATH_YAML}" >&2
  echo "        ./scripts/setup_simulation.sh를 다시 실행하세요." >&2
  exit 1
fi

GO2_PREFIX="$(ros2 pkg prefix unitree_go2_description)"
GO2_LEG_XACRO="${GO2_PREFIX}/share/unitree_go2_description/urdf/leg.xacro"
GO2_GAZEBO_XACRO="${GO2_PREFIX}/share/unitree_go2_description/urdf/unitree_go2_gazebo.xacro"
if ! grep -q '<param name="initial_value">1.0143535</param>' "${GO2_LEG_XACRO}" \
   || ! grep -q '<parameters>$(arg ros_control_file)</parameters>' "${GO2_GAZEBO_XACRO}"; then
  echo "[ERROR] 설치된 Go2 모델에 AG-CoNav 패치가 없습니다." >&2
  echo "        ./scripts/setup_simulation.sh를 다시 실행하세요." >&2
  exit 1
fi

# 이전 Gazebo/ROS 프로세스가 남아있으면 정리한다.
if pgrep -f "gz sim" > /dev/null 2>&1 || pgrep -f "ros_gz_bridge" > /dev/null 2>&1 || pgrep -f "ekf_node" > /dev/null 2>&1 || pgrep -f "robot_state_publisher" > /dev/null 2>&1; then
  echo "[WARN] 이전 시뮬레이션 프로세스가 남아있습니다. 정리합니다..."
  pkill -f "gz sim" || true
  pkill -f "ros2 launch" || true
  killall parameter_bridge robot_state_publisher ekf_node marker_server twist_mux joy_linux_node teleop_node quadruped_controller_node state_estimation_node navsat_transform_node tf_prefix_relay spawner component_container 2>/dev/null || true
  fastdds shm clean || true
  sleep 2
fi

exec ros2 launch agconav_bringup agconav_sim.launch.py "$@"
