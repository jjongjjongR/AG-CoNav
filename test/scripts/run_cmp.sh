#!/bin/bash
# unitree_guide vs RL robot_lab 맞대결 (단계식 경사로).
ROOT=$(cd "$(dirname "$0")/../.." && pwd); cd "$ROOT"
mkdir -p test/results/cmp
for i in 1 2 3; do
  for spec in "rl robot_lab robot_lab" "guide none guide"; do
    set -- $spec
    tag="cmp_$3_r$i"
    [ -s "test/results/cmp/$tag.json" ] && continue
    echo "=== $tag"
    timeout 900 ./test/scripts/run_op.sh test/worlds/ramp.sdf "$tag" ramp 0 "$1" "$2" 1.0 \
      "test/results/cmp/$tag.json" > /dev/null 2>&1
    python3 - "$tag" <<'PY'
import json,sys
t=sys.argv[1]
d=json.load(open('test/results/cmp/%s.json'%t))
print('  %-6s 최대각 %2s도  등반 %5.2f m  x %5.2f  roll %5.1f  pitch %5.1f  이탈 %4.2f'%(
  d.get('verdict'), d.get('max_angle','-'), d.get('climb') or 0, d.get('x_max') or 0,
  d.get('roll_max') or 0, d.get('pitch_max') or 0, d.get('drift_y') or 0))
PY
  done
done
echo "맞대결 완료"
