#!/usr/bin/env bash
# run_bag_replay_kalman.sh를 A/B/C 변형용으로 일반화 -- drone_elevation_mapper에
# 줄 추가 -p 오버라이드(R 보정/디스큐)만 다르고 나머지(F/verdictor/saver/capture)는
# #5 베이스라인과 완전히 동일하게 유지한다(공정한 비교를 위해).
#
#   run_variant_replay.sh <bag_dir> <태그> <결과.json> <extra ros2 -p 인자...>
BAG="$1"; TAG="$2"; OUT_JSON="$3"; shift 3
EXTRA_PARAMS=("$@")
DOMAIN=92
RATE=1.0
CAP_TIMEOUT=3000
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
pkill -f "ros2 bag play"               2>/dev/null
sleep 3

OUT="/tmp/trav_fn_$TAG"
rm -rf "$OUT"; mkdir -p "$OUT"

echo "[$(date '+%T')] variant=$TAG extra_params=${EXTRA_PARAMS[*]}"

ros2 run agconav_drone drone_elevation_mapper --ros-args \
  --params-file src/agconav_drone/config/drone_elevation_mapper.yaml \
  -p use_sim_time:=true "${EXTRA_PARAMS[@]}" > "$OUT/A.log" 2>&1 &
ros2 run agconav_traversability terrain_feature_calculator --ros-args \
  --params-file src/agconav_traversability/config/terrain_feature_calculator.yaml \
  -p use_sim_time:=true > "$OUT/F.log" 2>&1 &
ros2 run agconav_traversability traversability_verdictor --ros-args \
  -r __node:=traversability_verdictor_wheel \
  --params-file src/agconav_traversability/config/traversability_wheel.yaml \
  -p use_sim_time:=true > "$OUT/Fw.log" 2>&1 &
ros2 run agconav_traversability traversability_verdictor --ros-args \
  -r __node:=traversability_verdictor_leg \
  --params-file src/agconav_traversability/config/traversability_leg.yaml \
  -p use_sim_time:=true > "$OUT/Fl.log" 2>&1 &
rm -rf "$OUT/maps"
ros2 run agconav_drone elevation_map_saver --ros-args \
  --params-file src/agconav_drone/config/elevation_map_saver.yaml \
  -p output_directory:="$OUT/maps" -p use_sim_time:=true > "$OUT/S.log" 2>&1 &

sleep 6
python3 "$RR/capture_nav_fn.py" "$TAG" "$OUT_JSON" "$CAP_TIMEOUT" > "$OUT/cap.txt" 2>&1 &
CAP=$!
sleep 2
ros2 bag play "$BAG" --clock --rate "$RATE" > "$OUT/play.log" 2>&1
echo "[bag play 종료 $(date '+%T')] capture 대기 중..."
wait $CAP 2>/dev/null

pkill -f "elevation_[m]apper"          2>/dev/null
pkill -f "traversability_[v]erdictor"  2>/dev/null
pkill -f "terrain_[f]eature_calculator" 2>/dev/null
pkill -f "elevation_[m]ap_saver"       2>/dev/null
sleep 1
cat "$OUT/cap.txt"
echo "로그: $OUT   결과: $OUT_JSON"
