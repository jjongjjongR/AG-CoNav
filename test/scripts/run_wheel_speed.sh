#!/bin/bash
# wheel 속도 스윕. 두 가지를 한 번에 본다.
#   (1) 평지에서 명령 대비 실제 속도와 안정성 -> 최적 순항 속도
#   (2) 단차에서 접근 속도가 통과 여부를 가르는가
# 문서 12 는 0.8 m/s 하나로만 쟀는데, 단차는 접근 속도(운동량)에 민감하다.
ROOT=$(cd "$(dirname "$0")/../.." && pwd); cd "$ROOT"
REPS=${1:-2}
mkdir -p test/results/wspeed
for spec in "slope 0" "step 80" "step 100"; do
  set -- $spec
  KIND=$1; VAL=$2
  for spd in 0.5 0.8 1.2 1.6; do
    for i in $(seq 1 "$REPS"); do
      tag="v_${KIND}${VAL}_s${spd}_r$i"
      [ -s "test/results/wspeed/$tag.json" ] && continue
      echo "=== $tag"
      timeout 700 ./test/scripts/run_wheel.sh "test/worlds/${KIND}${VAL}.sdf" \
        "$tag" "$KIND" "$VAL" "$spd" "test/results/wspeed/$tag.json" > /dev/null 2>&1
      python3 - "$tag" <<'PY'
import json,sys
t=sys.argv[1]
try: d=json.load(open('test/results/wspeed/%s.json'%t))
except Exception as e: print('  읽기 실패',e); raise SystemExit
print('  %-8s 진출 %5.2f / %4.1f m  실제속도 %5.3f  기울기 %5.1f  이탈 %4.2f' % (
  d.get('verdict'), d.get('advance') or 0, d.get('need') or 0,
  d.get('speed_actual') or 0, d.get('tilt_max') or 0, d.get('drift_y') or 0))
PY
    done
  done
done
echo "속도 스윕 완료"
