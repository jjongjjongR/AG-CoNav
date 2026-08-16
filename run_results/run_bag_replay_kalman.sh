#!/usr/bin/env bash
# 재비행 bag(velocity, 5m AGL/4m/5mps)을 --clock으로 재생해, 이식된
# Module A(칼만필터+이상치방어 2겹) + Module F(terrain_feature_calculator,
# traversability_verdictor wheel/leg)를 실시간과 동일하게 라이브로 구독시켜
# 지도를 누적/발행시키고, capture_nav_fn.py로 wheel/leg FN%까지 채점한다.
# run_traversability_fn_v2.sh(feed_cloud.py 기반)와 달리 이번엔 합성 cloud를
# 안 만들고 bag의 원본 /drone/points+/tf+/tf_static+/drone/path_status를
# 그대로 재생한다 -- module_a:=false module_f:=false로 기록된 bag이라
# 이 4개 토픽만 실제로 메시지가 있고 나머지(elevation_map 등)는 비어있다.
#
#   run_bag_replay_kalman.sh <bag_dir> <태그> <결과.json> [ROS_DOMAIN_ID] [play_rate] [capture_timeout_s]
BAG="$1"; TAG="$2"; OUT_JSON="$3"; DOMAIN="${4:-92}"; RATE="${5:-1.0}"; CAP_TIMEOUT="${6:-3000}"
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

# use_sim_time:=true -- bag play --clock이 유일한 시간 소스(Gazebo 없음).
ros2 run agconav_drone drone_elevation_mapper --ros-args \
  --params-file src/agconav_drone/config/drone_elevation_mapper.yaml \
  -p use_sim_time:=true > "$OUT/A.log" 2>&1 &
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
