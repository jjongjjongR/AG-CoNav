#!/usr/bin/env python3
"""5m AGL 방법B 경로 재생성: 속도 5m/s, 스트립간격 4m (신규 세션, 4.5시간 자율 실행분).

generate_paths.py(2단계, 스트립간격 2/3/4m @ 4m/s)와 완전히 같은 지표면 모델/
안전검증 로직을 재사용하되, SPEED_MPS만 5.0으로 바꾸고 spacing=4.0 하나만
생성한다. 속도가 바뀌면 MAX_SLOPE = MAX_CLIMB_MPS/SPEED_MPS도 같이 바뀌므로
(같은 climb_speed_mps 예산 6.0 x 안전계수 0.7=4.2m/s를 더 빠른 순항속도로
나누면 허용 기울기가 더 완만해짐 -- 즉 건물 앞에서 더 일찍부터 상승을
시작해야 한다) 반드시 이 스크립트로 5m/s 기준 프로파일을 다시 계산해야 한다.
기존 4m/s 경로(path_100x100_5m_4m.yaml)는 건드리지 않고 별도 파일로 저장.
"""
from __future__ import annotations

import math
import time
import sys

import numpy as np
import yaml

sys.path.insert(0, "/home/hyunwoo-chae/AG-CoNav-test_main/run_results")
from surface_model import build_surface_grid, SurfaceModel, BOX  # noqa: E402
from generate_paths import (  # noqa: E402
    build_raw_waypoints, rate_limit_z, verify_and_fix, cumulative_metrics,
    count_turns, yaw_quat,
)
import generate_paths as gp  # noqa: E402

SPEED_MPS = 5.0
SPACING = 4.0
TAG = "4m_5mps"
TARGET_AGL = 5.0
MIN_MARGIN = 3.0
SAMPLE_STEP = 0.2

CLIMB_SPEED_BUDGET = 6.0
CLIMB_SAFETY_FACTOR = 0.7
MAX_CLIMB_MPS = CLIMB_SPEED_BUDGET * CLIMB_SAFETY_FACTOR
MAX_SLOPE = MAX_CLIMB_MPS / SPEED_MPS

# build_raw_waypoints/verify_and_fix가 generate_paths 모듈 전역(SPEED_MPS,
# ALONG_STEP, TARGET_AGL, MIN_MARGIN, SAMPLE_STEP, MAX_SUBDIV_ROUNDS)을 참조하므로
# 이번 실행 목적(5m/s)에 맞게 그 모듈의 전역을 덮어쓴다 -- generate_paths.py의
# 원본 SPEED_MPS(4.0)는 이 프로세스 안에서만 바뀌고 파일은 안 건드림.
gp.SPEED_MPS = SPEED_MPS
gp.TARGET_AGL = TARGET_AGL
gp.MIN_MARGIN = MIN_MARGIN
gp.SAMPLE_STEP = SAMPLE_STEP


def main():
    t_start = time.time()
    xs_grid, ys_grid, terrain, surface, buildings = build_surface_grid(
        resolution=0.5, cache_path="/home/hyunwoo-chae/AG-CoNav-test_main/run_results/surface_cache.npz")
    surf = SurfaceModel(xs_grid, ys_grid, surface)
    print(f"[surface] 건물 {len(buildings)}채, 지표면 고도 {surface.min():.3f}..{surface.max():.3f}")

    log = []
    xs, ys, zs, yaw_arr, n_strips = build_raw_waypoints(SPACING, surf)
    n_raw = len(xs)
    zs = rate_limit_z(xs, ys, zs, MAX_SLOPE)
    xs_f, ys_f, zs_f, n_corr = verify_and_fix(xs, ys, zs, surf, log)

    yaw_full = []
    raw_i = 0
    for k in range(len(xs_f)):
        if raw_i < len(xs) - 1 and abs(xs_f[k] - xs[raw_i + 1]) < 1e-9 and abs(ys_f[k] - ys[raw_i + 1]) < 1e-9:
            raw_i += 1
        yaw_full.append(yaw_arr[min(raw_i, len(yaw_arr) - 1)])
    yaw_full = np.array(yaw_full)

    # 최종 재검증
    d = np.hypot(np.diff(xs_f), np.diff(ys_f))
    min_clear = 1e9
    for i in range(len(xs_f) - 1):
        seg_len = d[i]
        if seg_len < 1e-6:
            continue
        n_samples = max(int(math.ceil(seg_len / SAMPLE_STEP)), 1)
        t = np.linspace(0.0, 1.0, n_samples + 1)
        sx = xs_f[i] + t * (xs_f[i + 1] - xs_f[i])
        sy = ys_f[i] + t * (ys_f[i + 1] - ys_f[i])
        sz = zs_f[i] + t * (zs_f[i + 1] - zs_f[i])
        ground = surf.height_at_array(sx, sy)
        min_clear = min(min_clear, float((sz - ground).min()))

    horiz = np.hypot(np.diff(xs_f), np.diff(ys_f))
    dz = np.abs(np.diff(zs_f))
    with np.errstate(divide="ignore", invalid="ignore"):
        req_vz = np.where(horiz > 1e-9, dz / (horiz / SPEED_MPS), 0.0)
    max_req_vz = float(req_vz.max()) if len(req_vz) else 0.0
    n_over_budget = int((req_vz > CLIMB_SPEED_BUDGET).sum())

    total_len = cumulative_metrics(xs_f, ys_f, zs_f)
    flight_time_s = total_len / SPEED_MPS
    n_turns = count_turns(yaw_full)

    waypoints = []
    for x, y, z, yaw in zip(xs_f, ys_f, zs_f, yaw_full):
        waypoints.append({
            "position": {"x": float(round(x, 4)), "y": float(round(y, 4)), "z": float(round(z, 4))},
            "orientation": {k: float(round(v, 6)) for k, v in yaw_quat(float(yaw)).items()},
        })

    out = {
        "frame_id": "map",
        "generation_method": "5m_AGL_method_B_max_terrain_building",
        "target_agl_m": TARGET_AGL,
        "min_safety_margin_m": MIN_MARGIN,
        "cruise_speed_mps": SPEED_MPS,
        "strip_spacing_m": SPACING,
        "along_strip_step_m": gp.ALONG_STEP,
        "num_waypoints": len(waypoints),
        "num_auto_corrections": n_corr,
        "climb_rate_limited_mps": MAX_CLIMB_MPS,
        "max_required_climb_mps": max_req_vz,
        "waypoints": waypoints,
    }
    out_path = ("/home/hyunwoo-chae/AG-CoNav-test_main/src/agconav_test_worlds/"
                f"config/path_100x100_5m_{TAG}.yaml")
    with open(out_path, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True)

    print(f"[{TAG}] raw={n_raw} final={len(waypoints)} corr={n_corr} "
          f"len={total_len:.1f}m time={flight_time_s:.1f}s turns={n_turns} "
          f"min_clear={min_clear:.3f}m max_req_vz={max_req_vz:.2f}m/s "
          f"(budget {CLIMB_SPEED_BUDGET}, over_budget_segs={n_over_budget}) -> {out_path}")

    report_path = "/home/hyunwoo-chae/AG-CoNav-test_main/run_results/path_5m_4m_5mps_safety_report.md"
    lines = []
    lines.append("# 5m AGL 방법B 경로 안전 검증 보고서 — 스트립간격 4m, 속도 5m/s\n\n")
    lines.append(f"- 목표 AGL: {TARGET_AGL} m, 순항 속도: {SPEED_MPS} m/s, "
                 f"최소 안전마진(사이 구간): {MIN_MARGIN} m\n")
    lines.append(f"- 지표면 모델: 0.5m 격자 캐시, 지형(height_map.png 쌍선형) vs "
                 f"건물(buildings.dae 연결요소 {len(buildings)}채, convex hull 포함판정 + 지붕Z) 중 최댓값 "
                 f"(`surface_model.py`, 2단계와 동일 재사용)\n")
    lines.append(f"- 스트립 내부 웨이포인트 간격: {gp.ALONG_STEP} m\n")
    lines.append(f"- 사이 구간 검증 샘플 간격: {SAMPLE_STEP} m\n")
    lines.append(f"- **상승률 제한**: 순항속도가 4m/s->5m/s로 바뀌어 같은 climb_speed_mps 예산"
                 f"({CLIMB_SPEED_BUDGET} x {CLIMB_SAFETY_FACTOR}={MAX_CLIMB_MPS:.2f}m/s)에서도 "
                 f"허용 기울기 상한이 |dz/ds|<={MAX_SLOPE:.4f}(4m/s 기준 경로의 "
                 f"{CLIMB_SPEED_BUDGET*CLIMB_SAFETY_FACTOR/4.0:.4f}보다 더 완만함)로 낮아져, "
                 f"건물 앞에서 더 일찍부터 상승을 시작하도록 재계산했다.\n\n")
    lines.append(f"## 스트립간격 {TAG}\n\n")
    lines.append(f"- 스트립 수: {n_strips}\n")
    lines.append(f"- 원시(보정 전) 웨이포인트: {n_raw}개\n")
    lines.append(f"- 최종 웨이포인트: {len(waypoints)}개\n")
    lines.append(f"- 총 3D 경로 길이: {total_len:.1f} m\n")
    lines.append(f"- 예상 순수 비행시간(등속 {SPEED_MPS} m/s 기준, 가감속 미포함): "
                 f"{flight_time_s:.1f} s ({flight_time_s/60:.1f} 분)\n")
    lines.append(f"- 턴(방향전환) 횟수: {n_turns}\n")
    lines.append(f"- 최소 지표면 클리어런스(전 구간, {SAMPLE_STEP}m 샘플): {min_clear:.3f} m "
                 f"(기준 {MIN_MARGIN} m 이상 {'충족' if min_clear >= MIN_MARGIN - 1e-6 else '미충족'})\n")
    lines.append(f"- 자동보정 횟수: {n_corr}\n")
    lines.append(f"- 요구 수직속도(등속 {SPEED_MPS} m/s 가정) 최대: {max_req_vz:.2f} m/s "
                 f"(설계 상한 {MAX_CLIMB_MPS:.2f} m/s, "
                 f"{'충족' if max_req_vz <= MAX_CLIMB_MPS + 1e-6 else '미충족'}) "
                 f"— 예산({CLIMB_SPEED_BUDGET}) 초과 세그먼트: {n_over_budget}개\n")
    if log:
        lines.append("\n<details><summary>보정 로그 전체 보기</summary>\n\n```\n")
        lines.append("\n".join(log))
        lines.append("\n```\n</details>\n")
    elapsed = time.time() - t_start
    lines.append(f"\n생성/검증 총 소요시간: {elapsed:.1f} s\n")
    with open(report_path, "w") as f:
        f.write("".join(lines))
    print(f"\n보고서 저장: {report_path}")


if __name__ == "__main__":
    main()
