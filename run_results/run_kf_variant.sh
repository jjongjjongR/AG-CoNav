#!/usr/bin/env bash
# run_bag_replay_kalman.sh를 파라미터 오버라이드 기반 변형(게이팅 임계값
# 스윕 등, 코드 수정 없이 launch -p로 바꿀 수 있는 실험)용으로 일반화.
# rate=2.0 고정(0단계에서 baseline #5 대비 오차 <0.3pp로 검증된 값).
#
#   run_kf_variant.sh <bag_dir> <태그> <결과.json> <extra ros2 -p 인자...>
BAG="$1"; TAG="$2"; OUT_JSON="$3"; shift 3
EXTRA_PARAMS=("$@")
DOMAIN=92
RATE=2.0
CAP_TIMEOUT=1800
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
  -p use_sim_time:=true \
  "${EXTRA_PARAMS[@]}" > "$OUT/A.log" 2>&1 &
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
