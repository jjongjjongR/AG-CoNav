#!/bin/bash
# 실험 1 등판 배치. 단계식 경사로에서 후보별 등판 한계를 잰다.
#   run_batch.sh [반복수] [속도...]
ROOT=$(cd "$(dirname "$0")/../.." && pwd); cd "$ROOT"
REPS=${1:-3}; shift || true
SPEEDS=${@:-1.0}
mkdir -p test/results/ramp
for spd in $SPEEDS; do
  for i in $(seq 1 "$REPS"); do
    # 이름 컨트롤러 정책
    for spec in "robot_lab rl robot_lab" "guide guide none" \
                "himloco rl himloco" "champ champ none"; do
      set -- $spec
      tag="r_$1_v${spd}_$i"
      [ -s "test/results/ramp/$tag.json" ] && continue
      echo "=== $tag"
      timeout 900 ./test/scripts/run_op.sh test/worlds/ramp.sdf "$tag" ramp 0 \
        "$2" "$3" "$spd" "test/results/ramp/$tag.json" > /dev/null 2>&1
      python3 - "$tag" <<'PY'
import json,sys
t=sys.argv[1]
try:
    d=json.load(open('test/results/ramp/%s.json'%t))
except Exception as e:
    print('  읽기 실패', e); raise SystemExit
print('  %-8s 최대각 %2s도  등반 %5.2f m  x %5.2f  roll %6.1f  이탈 %4.2f' % (
  d.get('verdict'), d.get('max_angle','-'), d.get('climb') or 0,
  d.get('x_max') or 0, d.get('roll_max') or 0, d.get('drift_y') or 0))
PY
    done
  done
done
echo "배치 완료"
