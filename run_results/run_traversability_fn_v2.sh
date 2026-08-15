#!/usr/bin/env bash
# run_traversability_fn.sh과 동일하나, 이번 5m AGL 4m/5mps cloud(2.67억 점,
# 84m 실험보다 훨씬 큼)를 feed_cloud.py가 200,000점/0.5s로 흘리는 데만
# ~11분이 걸려 원본 180초 캡처 타임아웃을 훨씬 넘긴다 -- capture_nav_fn.py
# 타임아웃을 넉넉히 늘려서 호출한다(그 외 로직은 원본과 동일, 무수정).
#
#   run_traversability_fn_v2.sh <cloud.npy> <태그> <결과.json> [ROS_DOMAIN_ID] [capture_timeout_s]
CLOUD="$1"; TAG="$2"; OUT_JSON="$3"; DOMAIN="${4:-91}"; CAP_TIMEOUT="${5:-1500}"
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
pkill -f "feed_cloud[.]py"             2>/dev/null
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
python3 "$RR/capture_nav_fn.py" "$TAG" "$OUT_JSON" "$CAP_TIMEOUT" > "$OUT/cap.txt" 2>&1 &
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
