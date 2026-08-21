#!/bin/bash
# leg 단차 시험. 단차는 지금까지 아무 문서에도 측정된 적이 없다(문서 12 §4).
#   run_step.sh [반복수] [속도]
ROOT=$(cd "$(dirname "$0")/../.." && pwd); cd "$ROOT"
REPS=${1:-3}; SPD=${2:-1.0}
mkdir -p test/results/step
for mm in 100 150 200 250 300; do
  for i in $(seq 1 "$REPS"); do
    for spec in "guide guide none" "robot_lab rl robot_lab"; do
      set -- $spec
      tag="s${mm}_$1_r$i"
      [ -s "test/results/step/$tag.json" ] && continue
      echo "=== $tag"
      timeout 900 ./test/scripts/run_op.sh "test/worlds/step${mm}.sdf" "$tag" \
        step "$mm" "$2" "$3" "$SPD" "test/results/step/$tag.json" > /dev/null 2>&1
      python3 - "$tag" <<'PY'
import json,sys
t=sys.argv[1]
try: d=json.load(open('test/results/step/%s.json'%t))
except Exception as e: print('  읽기 실패',e); raise SystemExit
print('  %-8s 진출 %5.2f m  올라섬 %-5s 등반 %5.3f  roll %6.1f  이탈 %4.2f' % (
  d.get('verdict'), d.get('advance') or 0, d.get('mounted'),
  d.get('climb') or 0, d.get('roll_max') or 0, d.get('drift_y') or 0))
PY
    done
  done
done
echo "단차 완료"
