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
# 기존 3000(50분)은 A/B/C 실행 때는 충분했으나, B'(preload+deskew) 첫
# 시도에서 이 bag(22GB, mcap 인덱스 없어 순차 스캔)의 point cloud 재생
# 자체가 원래도 burst 패턴으로 불안정한데(매퍼 없이도 실측 확인,
# PROGRESS.md 참고) preload+디스큐 연산까지 더해지며 50분을 넘겨 타임아웃
# 됐다. TF preload 캐시(pickle) 추가로 두 번째 실행부터는 훨씬 빨라지지만,
# 안전하게 넉넉히 늘려둔다.
# 6000(100분)도 부족했다 -- 재진단 결과 "hang"이 아니라 _check_data_received가
# elapsed<=2초일 때는 애초에 로그를 안 남기는 정상 설계였고(로그 침묵 =
# 정상 수신 중일 수 있음, PROGRESS.md 참고), drone_elevation_mapper는
# /drone/path_status(bag 맨 끝 1건)를 받아야만 지도를 발행하는 1회성
# 설계라, bag 재생 자체가 (burst 패턴 실측 확인, PROGRESS.md 참고)
# rate=1.0인데도 예상(37.6분)보다 몇 배 느려지면 그만큼 늦게 끝난다.
# 안전하게 3시간으로 늘려 "진짜 hang인지 그냥 느린 것뿐인지"를 이번에
# 확정한다.
CAP_TIMEOUT=10800
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
  -p deskew_tf_preload_bag_path:="$(pwd)/$BAG" \
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

# deskew_tf_preload_bag_path를 쓰는 실행(B')은 bag 전체 /tf(수십만 건)를
# spin 전에 다 읽느라 시작이 늦어질 수 있어, 그 preload가 끝났다는 로그가
# A.log에 찍히거나(디스큐 실행) 최대 60초까지 기다린 뒤에 bag play를
# 시작한다 -- 그 전에 시작하면 초반 point cloud를 놓칠 위험이 있다
# (비-디스큐 실행은 애초에 preload를 안 하므로 즉시 통과).
t_end=$((SECONDS + 60))
while [ $SECONDS -lt $t_end ]; do
  grep -q "deskew TF 사전로드 완료\|Accumulating" "$OUT/A.log" 2>/dev/null && break
  sleep 1
done
sleep 3
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
