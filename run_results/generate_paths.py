#!/usr/bin/env python3
"""2단계: 5m AGL 방법B 경로 3개(스트립간격 2m/3m/4m) 생성 + 안전검증.

- lawnmower(지그재그) 패턴, 스트립은 x방향, 스트립 간격만큼 y방향으로 이동.
- 각 스트립 내부 웨이포인트는 0.5m 간격(along_step)으로 촘촘히 찍는다.
- z = surface_model의 지표면 높이(x,y) + 5.0m을 요구 최소고도(z_req)로 삼되,
  건물 가장자리처럼 z_req가 짧은 거리에서 급변하는 곳에서는 velocity_path_follower.py
  가 실제로 따라갈 수 없는 상승률을 요구하게 된다(4단계 attempt1에서 실측: 0.5m
  이동 중 3.97m 상승 = 31.8 m/s 요구, climb_speed_mps 예산 6.0의 5배). 그래서
  z_req에 rate_limit_z()로 상승률 상한(설계상 4.2 m/s = 예산의 70%)을 적용해,
  항상 z_req 이상만 유지하면서(안전마진 절대 안 깨짐) 건물 진입 전부터 미리
  상승/하강하도록 만든 뒤 이 z를 실제 웨이포인트 고도로 쓴다.
- 안전검증: 웨이포인트 자체와, 웨이포인트 "사이" 구간을 0.2m 간격으로 재샘플링해
  선형보간 z가 "그 지점 실제 지표면 + 3.0m(최소 안전마진)" 이상인지 확인.
  위반 시 위반 구간을 절반으로 재분할(중점에 실제 필요 z로 웨이포인트 삽입)하는
  것을 위반이 없어질 때까지 반복(최대 8회 재분할/구간).
"""
from __future__ import annotations

import sys
import math
import time
import numpy as np
import yaml

sys.path.insert(0, "/home/hyunwoo-chae/AG-CoNav-test_main/run_results")
from surface_model import build_surface_grid, SurfaceModel, BOX  # noqa: E402

SPEED_MPS = 4.0
TARGET_AGL = 5.0
MIN_MARGIN = 3.0          # 사이 구간에서 허용하는 최소 클리어런스
ALONG_STEP = 0.5           # 스트립 내부 웨이포인트 간격 [m]
SAMPLE_STEP = 0.2          # 사이 구간 검증 샘플 간격 [m]
MAX_SUBDIV_ROUNDS = 8

# 4단계 attempt1(velocity 실비행)에서 z가 0.5m 수평 이동 중 3.97m 뛰는 구간이
# 나와(요구 수직속도 31.8 m/s) velocity_path_follower.py의 climb_speed_mps
# 예산(6.0)을 크게 초과, 팔로워가 안전중단했다. 전체 경로 스캔 결과 5,259개
# 세그먼트 중 170개(3.2%)가 이 예산을 넘었고 최악 93.4 m/s였다 — 건물 20채
# 전부의 가장자리에서 반복되는 구조적 문제. 게인 튜닝으로 해결 불가(경로 자체가
# 물리적으로 못 따라갈 상승률을 요구하는 것이므로).
#
# 대응: z-프로파일에 상승률 상한(rate limit)을 걸어, 건물 진입 전부터 미리
# 상승을 시작하고 지난 뒤 서서히 하강하게 만든다(항상 필요최소고도 "이상"만
# 유지하도록 위로만 완화하므로 안전마진은 절대 깨지지 않는다). 실제 컨트롤러가
# 명령을 그대로 못 따라가는 지연/오버슈트 여유를 두기 위해 climb_speed_mps
# 예산(6.0)의 70%만 설계 한계로 쓴다(사용자 승인 후 결정한 안전계수).
CLIMB_SPEED_BUDGET = 6.0   # velocity_path_follower.py의 climb_speed_mps 기본값
CLIMB_SAFETY_FACTOR = 0.7
MAX_CLIMB_MPS = CLIMB_SPEED_BUDGET * CLIMB_SAFETY_FACTOR   # 4.2 m/s
MAX_SLOPE = MAX_CLIMB_MPS / SPEED_MPS                       # dz/ds 상한 (무차원)

XA, XB, YA, YB = BOX


def yaw_quat(yaw):
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def build_raw_waypoints(spacing, surf: SurfaceModel):
    """스트립 lawnmower 원시 웨이포인트(코너 포함) 생성. surf+5.0m로 z를 채운다."""
    n_strips = int(round((YB - YA) / spacing)) + 1
    ys = np.linspace(YA, YB, n_strips)
    n_along = int(round((XB - XA) / ALONG_STEP)) + 1
    xs_fwd = np.linspace(XA, XB, n_along)
    xs_bwd = xs_fwd[::-1]

    pts = []  # (x, y, yaw)
    for i, y in enumerate(ys):
        xs = xs_fwd if i % 2 == 0 else xs_bwd
        yaw = 0.0 if i % 2 == 0 else math.pi
        for x in xs:
            pts.append((x, y, yaw))
        # 코너: 다음 스트립으로 옆이동 (마지막 스트립이면 생략)
        if i < len(ys) - 1:
            pts.append((xs[-1], ys[i + 1], yaw))

    xs_arr = np.array([p[0] for p in pts])
    ys_arr = np.array([p[1] for p in pts])
    yaw_arr = np.array([p[2] for p in pts])
    z_arr = surf.height_at_array(xs_arr, ys_arr) + TARGET_AGL
    return xs_arr, ys_arr, z_arr, yaw_arr, n_strips


def rate_limit_z(xs, ys, z_req, max_slope):
    """z_req(s) >= 만족하는 최소 slope-limited majorant.

    z(s) = max_j( z_req[j] - max_slope * |s - s_j| ), s는 누적 수평거리(arc-length).
    이 함수는 항상 z(s) >= z_req(s)이고 |dz/ds| <= max_slope를 만족하는, z_req를
    위에서 감싸는 가장 타이트한(=불필요하게 높이 날지 않는) 곡선이다. 표준
    "slope-limited majorant/dilation" 구성을 O(n) 두 번의 스캔으로 계산한다.
    """
    n = len(xs)
    ds = np.hypot(np.diff(xs), np.diff(ys))
    s = np.concatenate([[0.0], np.cumsum(ds)])

    fwd = np.empty(n)
    running = -np.inf
    for i in range(n):
        running = max(running, z_req[i] + max_slope * s[i])
        fwd[i] = running - max_slope * s[i]

    bwd = np.empty(n)
    running = -np.inf
    for i in range(n - 1, -1, -1):
        running = max(running, z_req[i] - max_slope * s[i])
        bwd[i] = running + max_slope * s[i]

    return np.maximum(fwd, bwd)


def verify_and_fix(xs, ys, zs, surf: SurfaceModel, log):
    """세그먼트별로 0.2m 샘플링 검증, 위반 시 재분할. (xs,ys,zs) in-place 확장 반환."""
    xs = list(xs)
    ys = list(ys)
    zs = list(zs)
    total_corrections = 0
    i = 0
    while i < len(xs) - 1:
        x0, y0, z0 = xs[i], ys[i], zs[i]
        x1, y1, z1 = xs[i + 1], ys[i + 1], zs[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len < 1e-6:
            i += 1
            continue

        n_samples = max(int(math.ceil(seg_len / SAMPLE_STEP)), 1)
        t = np.linspace(0.0, 1.0, n_samples + 1)
        sx = x0 + t * (x1 - x0)
        sy = y0 + t * (y1 - y0)
        sz = z0 + t * (z1 - z0)
        ground = surf.height_at_array(sx, sy)
        clearance = sz - ground
        worst_idx = int(np.argmin(clearance))
        worst_clear = clearance[worst_idx]

        if worst_clear >= MIN_MARGIN - 1e-9:
            i += 1
            continue

        # 위반: 그 지점에 실제 필요한 z(그 지점 지표면 + TARGET_AGL)로 웨이포인트를 삽입해
        # 세그먼트를 둘로 쪼갠다. t=0 또는 t=1(=기존 웨이포인트 자체)이면 그 웨이포인트의
        # z 자체를 올린다.
        total_corrections += 1
        tw = t[worst_idx]
        need_z = ground[worst_idx] + TARGET_AGL
        if tw <= 1e-9:
            zs[i] = max(zs[i], need_z)
            log.append(f"  보정#{total_corrections}: seg[{i}] 시작점 자체 z 상향 "
                       f"{z0:.3f}->{zs[i]:.3f} (지표면 {ground[worst_idx]:.3f})")
            continue
        if tw >= 1 - 1e-9:
            zs[i + 1] = max(zs[i + 1], need_z)
            log.append(f"  보정#{total_corrections}: seg[{i}] 끝점 자체 z 상향 "
                       f"{z1:.3f}->{zs[i+1]:.3f} (지표면 {ground[worst_idx]:.3f})")
            continue

        new_x, new_y = float(sx[worst_idx]), float(sy[worst_idx])
        xs.insert(i + 1, new_x)
        ys.insert(i + 1, new_y)
        zs.insert(i + 1, float(need_z))
        log.append(f"  보정#{total_corrections}: seg[{i}] t={tw:.3f}에 웨이포인트 삽입 "
                   f"({new_x:.2f},{new_y:.2f}) z={need_z:.3f} "
                   f"(위반 클리어런스 {worst_clear:.3f}m < {MIN_MARGIN}m)")
        # 같은 위치(i, i+1 재분할된 것) 재검증 위해 i는 그대로 둔다.
    return np.array(xs), np.array(ys), np.array(zs), total_corrections


def cumulative_metrics(xs, ys, zs):
    d3 = np.hypot(np.hypot(np.diff(xs), np.diff(ys)), np.diff(zs))
    total_len = float(d3.sum())
    return total_len


def count_turns(yaw_arr):
    # yaw가 바뀌는 지점(코너)의 수
    changes = np.sum(np.abs(np.diff(yaw_arr)) > 1e-6)
    return int(changes)


def main():
    t_start = time.time()
    xs_grid, ys_grid, terrain, surface, buildings = build_surface_grid(
        resolution=0.5, cache_path="/home/hyunwoo-chae/AG-CoNav-test_main/run_results/surface_cache.npz")
    surf = SurfaceModel(xs_grid, ys_grid, surface)
    print(f"[surface] 건물 {len(buildings)}채, 지표면 고도 {surface.min():.3f}..{surface.max():.3f}")

    report_lines = []
    report_lines.append("# 5m AGL 방법B 경로 안전 검증 보고서\n")
    report_lines.append(f"- 목표 AGL: {TARGET_AGL} m, 순항 속도: {SPEED_MPS} m/s, "
                        f"최소 안전마진(사이 구간): {MIN_MARGIN} m\n")
    report_lines.append(f"- 지표면 모델: 0.5m 격자 캐시, 지형(height_map.png 쌍선형) vs "
                        f"건물(buildings.dae 연결요소 20채, convex hull 포함판정 + 지붕Z) 중 최댓값\n")
    report_lines.append(f"- 스트립 내부 웨이포인트 간격: {ALONG_STEP} m (시작값, 촘촘히 고정 적용)\n")
    report_lines.append(f"- 사이 구간 검증 샘플 간격: {SAMPLE_STEP} m\n")
    report_lines.append(f"- **상승률 제한(v2, 4단계 attempt1 발산 이후 추가)**: z 프로파일에 "
                        f"slope-limited majorant(2-pass max-plus)를 적용해 항상 필요최소고도 "
                        f"이상만 유지하면서 |dz/ds| <= {MAX_SLOPE:.3f}(순항 {SPEED_MPS} m/s 기준 "
                        f"수직 {MAX_CLIMB_MPS:.2f} m/s)를 보장한다. velocity_path_follower.py의 "
                        f"climb_speed_mps 예산({CLIMB_SPEED_BUDGET})의 {int(CLIMB_SAFETY_FACTOR*100)}%만 "
                        f"설계 한계로 써서 실제 컨트롤러 추종 지연/오버슈트 여유를 둔다.\n\n")

    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="+", default=["2m", "3m", "4m"],
                    help="재생성할 간격 태그만 선택 (예: --only 4m). 기본은 전부.")
    args = ap.parse_args()

    for spacing, tag in [(2.0, "2m"), (3.0, "3m"), (4.0, "4m")]:
        if tag not in args.only:
            continue
        log = []
        xs, ys, zs, yaw_arr, n_strips = build_raw_waypoints(spacing, surf)
        n_raw = len(xs)
        zs = rate_limit_z(xs, ys, zs, MAX_SLOPE)
        xs_f, ys_f, zs_f, n_corr = verify_and_fix(xs, ys, zs, surf, log)

        # yaw: 보정으로 추가된 점들은 직전 점의 yaw를 그대로 물려받는다(방향 유지).
        # 원시 배열과 보정후 배열 길이가 다르므로, x/y로 직전 원시 인덱스를 재추적.
        yaw_full = []
        raw_i = 0
        for k in range(len(xs_f)):
            if raw_i < len(xs) - 1 and abs(xs_f[k] - xs[raw_i + 1]) < 1e-9 and abs(ys_f[k] - ys[raw_i + 1]) < 1e-9:
                raw_i += 1
            yaw_full.append(yaw_arr[min(raw_i, len(yaw_arr) - 1)])
        yaw_full = np.array(yaw_full)

        # 최종 재검증 (0.2m 샘플, 전 구간 최소 클리어런스)
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

        # 세그먼트별 요구 수직속도(등속 SPEED_MPS 가정) 점검 -- attempt1에서 이걸
        # 안 봐서 30 m/s대 요구 구간을 놓쳤던 결함의 재발 방지용 체크.
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
            "strip_spacing_m": spacing,
            "along_strip_step_m": ALONG_STEP,
            "num_waypoints": len(waypoints),
            "num_auto_corrections": n_corr,
            "climb_rate_limited_mps": MAX_CLIMB_MPS,
            "max_required_climb_mps": max_req_vz,
            "waypoints": waypoints,
        }
        out_path = (f"/home/hyunwoo-chae/AG-CoNav-test_main/src/agconav_test_worlds/"
                    f"config/path_100x100_5m_{tag}.yaml")
        with open(out_path, "w") as f:
            yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True)

        print(f"[{tag}] raw={n_raw} final={len(waypoints)} corr={n_corr} "
              f"len={total_len:.1f}m time={flight_time_s:.1f}s turns={n_turns} "
              f"min_clear={min_clear:.3f}m max_req_vz={max_req_vz:.2f}m/s "
              f"(budget {CLIMB_SPEED_BUDGET}, over_budget_segs={n_over_budget}) -> {out_path}")

        report_lines.append(f"## 스트립간격 {tag}\n\n")
        report_lines.append(f"- 스트립 수: {n_strips}\n")
        report_lines.append(f"- 원시(보정 전) 웨이포인트: {n_raw}개\n")
        report_lines.append(f"- 최종 웨이포인트: {len(waypoints)}개\n")
        report_lines.append(f"- 총 3D 경로 길이: {total_len:.1f} m\n")
        report_lines.append(f"- 예상 순수 비행시간(등속 {SPEED_MPS} m/s 기준, 가감속 미포함): "
                            f"{flight_time_s:.1f} s ({flight_time_s/60:.1f} 분)\n")
        report_lines.append(f"- 턴(방향전환) 횟수: {n_turns}\n")
        report_lines.append(f"- 최소 지표면 클리어런스(전 구간, 0.2m 샘플): {min_clear:.3f} m "
                            f"(기준 {MIN_MARGIN} m 이상 {'충족' if min_clear >= MIN_MARGIN - 1e-6 else '미충족'})\n")
        report_lines.append(f"- 자동보정 횟수: {n_corr}\n")
        report_lines.append(f"- 요구 수직속도(등속 {SPEED_MPS} m/s 가정) 최대: {max_req_vz:.2f} m/s "
                            f"(설계 상한 {MAX_CLIMB_MPS:.2f} m/s = climb_speed_mps 예산 "
                            f"{CLIMB_SPEED_BUDGET} x 안전계수 {CLIMB_SAFETY_FACTOR}, "
                            f"{'충족' if max_req_vz <= MAX_CLIMB_MPS + 1e-6 else '미충족'}) "
                            f"— 예산({CLIMB_SPEED_BUDGET}) 초과 세그먼트: {n_over_budget}개\n")
        if log:
            report_lines.append("\n<details><summary>보정 로그 전체 보기</summary>\n\n```\n")
            report_lines.append("\n".join(log))
            report_lines.append("\n```\n</details>\n")
        report_lines.append("\n")

    elapsed = time.time() - t_start
    report_lines.append(f"\n생성/검증 총 소요시간: {elapsed:.1f} s\n")

    report_path = "/home/hyunwoo-chae/AG-CoNav-test_main/run_results/path_5m_safety_report.md"
    with open(report_path, "w") as f:
        f.write("".join(report_lines))
    print(f"\n보고서 저장: {report_path}")


if __name__ == "__main__":
    main()
