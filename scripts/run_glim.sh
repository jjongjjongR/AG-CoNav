#!/usr/bin/env bash
# Run GLIM in Docker against a recorded AG-CoNav drone bag.
#
# The image is the jazzy build, matching the host distro, so the bag can be
# played either on the host or inside the container -- the Humble images force
# the latter, because a cross-distro DDS pair does not exchange these types
# reliably. Here the bag is mounted and played inside anyway, which keeps the
# run self-contained and independent of what is running on the host.
#
#   ./scripts/run_glim.sh                       # default bag, GPU
#   ./scripts/run_glim.sh bags/drone_scan_100x100
#   ./scripts/run_glim.sh bags/drone_scan_100x100 --cpu
#
# Config comes from ./glim_config, which was extracted from the image and then
# edited for this robot (topics, QoS, T_lidar_imu). Edits there take effect on
# the next run without rebuilding anything.
set -euo pipefail

BAG="${1:-bags/drone_scan_100x100}"
shift || true
GPU_ARGS=(--gpus all)
IMAGE="koide3/glim_ros2:jazzy_cuda13.1"
for a in "$@"; do
  if [ "$a" = "--cpu" ]; then GPU_ARGS=(); fi
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_ABS="$(cd "$REPO" && cd "$(dirname "$BAG")" && pwd)/$(basename "$BAG")"
CFG="$REPO/glim_config"

if [ ! -d "$BAG_ABS" ]; then
  echo "bag 디렉터리가 없습니다: $BAG_ABS" >&2
  exit 1
fi
if [ ! -d "$CFG" ]; then
  echo "glim_config 가 없습니다: $CFG" >&2
  exit 1
fi

# `docker` needs the docker group; this session may not have picked it up yet.
DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
  if id -nG | tr ' ' '\n' | grep -qx docker; then
    DOCKER=(sg docker -c)
  else
    echo "docker 권한이 없습니다. 'newgrp docker' 후 다시 시도하거나 sudo로 실행하세요." >&2
    exit 1
  fi
fi

CMD="ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config & \
     sleep 8 && ros2 bag play /data/$(basename "$BAG_ABS") --clock && sleep 5"

echo "bag    : $BAG_ABS"
echo "config : $CFG"
echo "image  : $IMAGE  ${GPU_ARGS[*]:-(CPU)}"

if [ "${DOCKER[0]}" = "sg" ]; then
  sg docker -c "docker run --rm -it --net=host --ipc=host --pid=host \
    ${GPU_ARGS[*]} -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v '$CFG':/glim/config -v '$BAG_ABS':/data/$(basename "$BAG_ABS") \
    $IMAGE bash -lc \"$CMD\""
else
  docker run --rm -it --net=host --ipc=host --pid=host \
    "${GPU_ARGS[@]}" -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$CFG":/glim/config -v "$BAG_ABS":/data/"$(basename "$BAG_ABS")" \
    "$IMAGE" bash -lc "$CMD"
fi
