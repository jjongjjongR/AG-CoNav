#!/usr/bin/env bash
# 누적 점군 하나를 모듈 A -> 모듈 F 체인에 태워 주행성 수치를 뽑는다.
#
# 모듈은 전부 리포지토리 원본을 그대로 쓴다. 바뀌는 것은 입력 점군뿐이다.
#
#   run_traversability.sh <cloud.npy> <태그> [ROS_DOMAIN_ID]
#
# verdictor 두 개는 노드 이름을 명시해야 한다. yaml 키가
# traversability_verdictor_wheel / _leg 라서, 이름을 주지 않으면 둘 다 기본값
# (wheel)으로 떠서 leg 지도가 나오지 않는다.
# set -u 는 쓰지 않는다. ROS의 setup.bash 가 미설정 변수를 참조해서 즉시 죽는다.
CLOUD="$1"; TAG="$2"; DOMAIN="${3:-91}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
cd "$ROOT"

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="$DOMAIN"

# 이전 실행이 남긴 노드가 있으면 같은 토픽에 옛 결과를 latched로 계속 흘린다.
pkill -f "elevation_[m]apper"          2>/dev/null
pkill -f "traversability_[v]erdictor"  2>/dev/null
pkill -f "terrain_[f]eature_calculator" 2>/dev/null
pkill -f "elevation_[m]ap_saver"       2>/dev/null
sleep 3

# 로그는 남긴다. 실패 원인이 여기에만 남는데 지워버리면 진단이 불가능하다.
OUT="/tmp/trav_$TAG"
rm -rf "$OUT"; mkdir -p "$OUT"

ros2 run agconav_drone drone_elevation_mapper --ros-args \
  --params-file src/agconav_drone/config/drone_elevation_mapper.yaml \
  -p use_sim_time:=false > "$OUT/A.log" 2>&1 &
ros2 run agconav_traversability terrain_feature_calculator --ros-args \
  --params-file src/agconav_traversability/config/terrain_feature_calculator.yaml \
  -p use_sim_time:=false > "$OUT/F.log" 2>&1 &
ros2 run agconav_traversability traversability_verdictor --ros-args \
  -r __node:=traversability_verdictor_wheel \
  --params-file src/agconav_traversability/config/traversability_wheel.yaml \
  -p use_sim_time:=false > "$OUT/Fw.log" 2>&1 &
ros2 run agconav_traversability traversability_verdictor --ros-args \
  -r __node:=traversability_verdictor_leg \
  --params-file src/agconav_traversability/config/traversability_leg.yaml \
  -p use_sim_time:=false > "$OUT/Fl.log" 2>&1 &
# 모듈 F의 트리거(/drone/elevation_map_status)는 saver가 발행한다.
# 저장 디렉터리가 이미 있으면 saver가 실패하고 트리거도 안 나가므로 매번 비운다.
rm -rf "$OUT/maps"
ros2 run agconav_drone elevation_map_saver --ros-args \
  --params-file src/agconav_drone/config/elevation_map_saver.yaml \
  -p output_directory:="$OUT/maps" -p use_sim_time:=false > "$OUT/S.log" 2>&1 &

sleep 6
python3 "$HERE/capture_nav.py" "$TAG" 120 > "$OUT/cap.txt" 2>&1 &
CAP=$!
sleep 2
python3 "$HERE/feed_cloud.py" "$CLOUD" > "$OUT/feed.log" 2>&1
wait $CAP 2>/dev/null

pkill -f "elevation_[m]apper"          2>/dev/null
pkill -f "traversability_[v]erdictor"  2>/dev/null
pkill -f "terrain_[f]eature_calculator" 2>/dev/null
pkill -f "elevation_[m]ap_saver"       2>/dev/null
sleep 1
cat "$OUT/cap.txt"
echo "로그: $OUT"
