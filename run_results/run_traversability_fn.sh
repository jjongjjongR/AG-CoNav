#!/usr/bin/env bash
# run_traversability.sh(원본, src/agconav_test_worlds/scripts/)와 동일한 패턴이지만
# capture_nav.py 대신 capture_nav_fn.py를 써서 GT 대비 wheel/leg FN%까지 뽑는다.
# 모듈 A/F 노드 자체는 원본 그대로(무수정) 사용한다.
#
#   run_traversability_fn.sh <cloud.npy> <태그> <결과.json> [ROS_DOMAIN_ID]
CLOUD="$1"; TAG="$2"; OUT_JSON="$3"; DOMAIN="${4:-91}"
ROOT="/home/hyunwoo-chae/AG-CoNav-test_main"
RR="$ROOT/run_results"
cd "$ROOT"

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="$DOMAIN"

pkill -f "elevation_[m]apper"          2>/dev/null
pkill -f "traversability_[v]erdictor"  2>/dev/null
pkill -f "terrain_[f]eature_calculator" 2>/dev/null
pkill -f "elevation_[m]ap_saver"       2>/dev/null
sleep 3

OUT="/tmp/trav_fn_$TAG"
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
rm -rf "$OUT/maps"
ros2 run agconav_drone elevation_map_saver --ros-args \
  --params-file src/agconav_drone/config/elevation_map_saver.yaml \
  -p output_directory:="$OUT/maps" -p use_sim_time:=false > "$OUT/S.log" 2>&1 &

sleep 6
python3 "$RR/capture_nav_fn.py" "$TAG" "$OUT_JSON" 180 > "$OUT/cap.txt" 2>&1 &
CAP=$!
sleep 2
python3 "$ROOT/src/agconav_test_worlds/scripts/feed_cloud.py" "$CLOUD" > "$OUT/feed.log" 2>&1
wait $CAP 2>/dev/null

pkill -f "elevation_[m]apper"          2>/dev/null
pkill -f "traversability_[v]erdictor"  2>/dev/null
pkill -f "terrain_[f]eature_calculator" 2>/dev/null
pkill -f "elevation_[m]ap_saver"       2>/dev/null
sleep 1
cat "$OUT/cap.txt"
echo "로그: $OUT   결과: $OUT_JSON"
