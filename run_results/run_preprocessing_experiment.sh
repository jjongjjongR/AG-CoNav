#!/usr/bin/env bash
# gicp-preprocessing 실험 오케스트레이션: build(전처리 변형) -> score 를
# 순차로 돌린다. run_traversability_fn_v2.sh가 시작할 때 프로세스 이름으로
# pkill을 하기 때문에(도메인 무관) 두 채점을 동시에 돌리면 서로 죽일 위험이
# 있어 일부러 순차 실행한다.
#
#   run_preprocessing_experiment.sh <bag_dir> <variant1> [variant2 ...]
set -o pipefail
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

BAG="$1"; shift
VARIANTS=("$@")
OUTDIR="/tmp/gicp_preproc"
mkdir -p "$OUTDIR"
LOG="run_results/logs/preprocessing_experiment.log"

log(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

for tag in "${VARIANTS[@]}"; do
  log "BUILD start: $tag"
  python3 run_results/build_methodB_cloud_preprocess.py \
    "$BAG" "$OUTDIR/baseline_${tag}.npy" "$OUTDIR/methodb_${tag}.npy" "$tag" \
    >> "$LOG" 2>&1
  BS=$?
  log "BUILD done: $tag (exit $BS)"

  log "SCORE start: methodb_$tag"
  bash run_results/run_traversability_fn_v2.sh \
    "$OUTDIR/methodb_${tag}.npy" "methodb_${tag}" "$OUTDIR/methodb_${tag}_fn.json" 91 1500 \
    >> "$LOG" 2>&1
  SS=$?
  log "SCORE done: methodb_$tag (exit $SS)"

  # baseline.npy는 전처리와 무관(전부 동일) -- 첫 variant에서만 보관, 이후는 지워서 디스크 절약
  if [ "$tag" != "${VARIANTS[0]}" ]; then
    rm -f "$OUTDIR/baseline_${tag}.npy"
  fi
  rm -f "$OUTDIR/methodb_${tag}.npy"
done

log "ALL DONE: ${VARIANTS[*]}"
echo "ALL_DONE" > "$OUTDIR/DONE_$(IFS=_; echo "${VARIANTS[*]}")"
