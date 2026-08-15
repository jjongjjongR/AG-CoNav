#!/usr/bin/env python3
"""2m/3m은 사용자 지시로 실험 제외 확정 -- yaml 파일은 절대 재생성/재기록하지
않는다. 이 스크립트는 4m 재생성(rate-limiter 추가)으로 날아간 safety report의
2m/3m 절만, 원래 만들었을 때와 동일한(rate-limit 이전) 로직으로 다시 계산해
텍스트만 복원한다. generate_paths.py를 import해 함수만 재사용하되, rate_limit_z는
호출하지 않고 yaml.safe_dump도 하지 않는다.
"""
import sys
import math
import numpy as np

sys.path.insert(0, "/home/hyunwoo-chae/AG-CoNav-test_main/run_results")
import generate_paths as gp  # noqa: E402
from surface_model import build_surface_grid, SurfaceModel  # noqa: E402

xs_grid, ys_grid, terrain, surface, buildings = build_surface_grid(
    resolution=0.5, cache_path="/home/hyunwoo-chae/AG-CoNav-test_main/run_results/surface_cache.npz")
surf = SurfaceModel(xs_grid, ys_grid, surface)

sections = []
for spacing, tag in [(2.0, "2m"), (3.0, "3m")]:
    log = []
    xs, ys, zs, yaw_arr, n_strips = gp.build_raw_waypoints(spacing, surf)
    n_raw = len(xs)
    # rate_limit_z 호출하지 않음 -- 원래(v1) 로직 그대로 재현
    xs_f, ys_f, zs_f, n_corr = gp.verify_and_fix(xs, ys, zs, surf, log)

    yaw_full = []
    raw_i = 0
    for k in range(len(xs_f)):
        if raw_i < len(xs) - 1 and abs(xs_f[k] - xs[raw_i + 1]) < 1e-9 and abs(ys_f[k] - ys[raw_i + 1]) < 1e-9:
            raw_i += 1
        yaw_full.append(yaw_arr[min(raw_i, len(yaw_arr) - 1)])
    yaw_full = np.array(yaw_full)

    d = np.hypot(np.diff(xs_f), np.diff(ys_f))
    min_clear = 1e9
    for i in range(len(xs_f) - 1):
        seg_len = d[i]
        if seg_len < 1e-6:
            continue
        n_samples = max(int(math.ceil(seg_len / gp.SAMPLE_STEP)), 1)
        t = np.linspace(0.0, 1.0, n_samples + 1)
        sx = xs_f[i] + t * (xs_f[i + 1] - xs_f[i])
        sy = ys_f[i] + t * (ys_f[i + 1] - ys_f[i])
        sz = zs_f[i] + t * (zs_f[i + 1] - zs_f[i])
        ground = surf.height_at_array(sx, sy)
        min_clear = min(min_clear, float((sz - ground).min()))

    total_len = gp.cumulative_metrics(xs_f, ys_f, zs_f)
    flight_time_s = total_len / gp.SPEED_MPS
    n_turns = gp.count_turns(yaw_full)

    lines = []
    lines.append(f"## 스트립간격 {tag}  *(2단계 원안 -- 실험 제외 확정, 참고용 보존)*\n\n")
    lines.append(f"- 스트립 수: {n_strips}\n")
    lines.append(f"- 원시(보정 전) 웨이포인트: {n_raw}개\n")
    lines.append(f"- 최종 웨이포인트: {len(xs_f)}개\n")
    lines.append(f"- 총 3D 경로 길이: {total_len:.1f} m\n")
    lines.append(f"- 예상 순수 비행시간(등속 {gp.SPEED_MPS} m/s 기준, 가감속 미포함): "
                f"{flight_time_s:.1f} s ({flight_time_s/60:.1f} 분)\n")
    lines.append(f"- 턴(방향전환) 횟수: {n_turns}\n")
    lines.append(f"- 최소 지표면 클리어런스(전 구간, 0.2m 샘플): {min_clear:.3f} m "
                f"(기준 {gp.MIN_MARGIN} m 이상 {'충족' if min_clear >= gp.MIN_MARGIN - 1e-6 else '미충족'})\n")
    lines.append(f"- 자동보정 횟수: {n_corr}\n")
    lines.append(f"- **주의**: 이 절은 상승률 제한(rate-limiter) 적용 *이전* 로직으로 계산됐다"
                f"(4단계 attempt1에서 발산 원인이 밝혀진 뒤 4m에만 rate-limiter를 추가했고, "
                f"2m/3m 실험은 그 이전에 이미 제외가 확정돼 rate-limiter 버전으로 재생성하지 "
                f"않았다). 즉 이 경로들도 실제 velocity 비행에 썼다면 4m attempt1과 동일한 "
                f"상승률 문제를 겪었을 것으로 추정되나, 실비행을 시도하지 않았으므로 확인된 "
                f"사실은 아니다.\n")
    if log:
        lines.append("\n<details><summary>보정 로그 전체 보기</summary>\n\n```\n")
        lines.append("\n".join(log))
        lines.append("\n```\n</details>\n")
    lines.append("\n")
    sections.append("".join(lines))
    print(f"[{tag}] 복원 완료: final={len(xs_f)} corr={n_corr} len={total_len:.1f}m "
         f"min_clear={min_clear:.3f}m")

with open("/home/hyunwoo-chae/AG-CoNav-test_main/run_results/_2m_3m_sections.txt", "w") as f:
    f.write("".join(sections))
print("저장: run_results/_2m_3m_sections.txt")
