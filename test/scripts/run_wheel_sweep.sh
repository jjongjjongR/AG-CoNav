#!/bin/bash
# wheel(A300) 경사·단차 스윕. 문서 12(2026-08-16)의 재확인이다.
#   run_wheel_sweep.sh [반복수] [속도]
ROOT=$(cd "$(dirname "$0")/../.." && pwd); cd "$ROOT"
REPS=${1:-3}; SPD=${2:-0.8}
mkdir -p test/results/wheel
# 문서 12 의 결론은 등판 10도 통과 / 15도 실패, 단차 100 mm 통과 / 120 mm 실패다.
# 그 경계를 사이에 두고 다시 훑는다(12도, 110 mm 를 새로 넣어 해상도를 높였다).
for spec in "slope 0" "slope 5" "slope 10" "slope 12" "slope 15" "slope 20" \
            "step 80" "step 100" "step 110" "step 120" "step 150"; do
  set -- $spec
  KIND=$1; VAL=$2
  for i in $(seq 1 "$REPS"); do
    tag="w_${KIND}${VAL}_r$i"
    [ -s "test/results/wheel/$tag.json" ] && continue
    echo "=== $tag"
    timeout 900 ./test/scripts/run_wheel.sh "test/worlds/${KIND}${VAL}.sdf" \
      "$tag" "$KIND" "$VAL" "$SPD" "test/results/wheel/$tag.json" > /dev/null 2>&1
    python3 - "$tag" <<'PY'
import json,sys
t=sys.argv[1]
try: d=json.load(open('test/results/wheel/%s.json'%t))
except Exception as e: print('  읽기 실패',e); raise SystemExit
print('  %-8s 진출 %5.2f / %4.1f m  속도 %5.3f  기울기 %5.1f  이탈 %4.2f' % (
  d.get('verdict'), d.get('advance') or 0, d.get('need') or 0,
  d.get('speed_actual') or 0, d.get('tilt_max') or 0, d.get('drift_y') or 0))
PY
  done
done
echo "wheel 스윕 완료"
