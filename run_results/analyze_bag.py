#!/usr/bin/env python3
"""드라이런/실비행 bag에서 /drone/elevation_map(GridMap, latched 1회)을 꺼내
지표면 모델과 비교해 커버리지/높이오차/자기반사 여부를 진단한다.

사용: analyze_bag.py <bag_dir> [--out <report.md>] [--tag <이름>]
"""
import argparse
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, "/home/hyunwoo-chae/AG-CoNav-test_main/run_results")
from surface_model import build_surface_grid, SurfaceModel, BOX  # noqa: E402


def layer_to_array(grid_map, layer):
    data = grid_map.data[grid_map.layers.index(layer)]
    size_y = data.layout.dim[0].size
    size_x = data.layout.dim[1].size
    array = np.asarray(data.data, dtype=np.float32).reshape(size_y, size_x).T
    array = np.roll(array, -grid_map.outer_start_index, axis=0)
    return np.roll(array, -grid_map.inner_start_index, axis=1)


def read_topic_messages(bag_dir, topic):
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topics_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_types}
    if topic not in type_map:
        return []
    msg_type = get_message(type_map[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while reader.has_next():
        (t, data, stamp) = reader.read_next()
        out.append(deserialize_message(data, msg_type))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir")
    ap.add_argument("--out", default=None)
    ap.add_argument("--tag", default=None)
    args = ap.parse_args()
    tag = args.tag or args.bag_dir

    path_status = read_topic_messages(args.bag_dir, "/drone/path_status")
    elev_msgs = read_topic_messages(args.bag_dir, "/drone/elevation_map")

    lines = [f"# 드라이런 결과 — {tag}\n\n", f"- bag: `{args.bag_dir}`\n"]
    lines.append(f"- /drone/path_status 메시지 수: {len(path_status)}"
                 + (f" (마지막 값: {path_status[-1].data})" if path_status else " (없음!)") + "\n")
    lines.append(f"- /drone/elevation_map 메시지 수: {len(elev_msgs)}\n\n")

    if not elev_msgs:
        lines.append("**elevation_map이 발행되지 않았다 — 비정상 종료 가능성.**\n")
        report = "".join(lines)
        print(report)
        if args.out:
            with open(args.out, "w") as f:
                f.write(report)
        return 1

    gm = elev_msgs[-1]
    elev = layer_to_array(gm, "elevation")
    n_valid = np.isfinite(elev).sum()
    n_total = elev.size
    coverage = 100.0 * n_valid / n_total
    valid = elev[np.isfinite(elev)]

    # GridMapInfo: pose.position is the map CENTER, length_x/length_y are the extents.
    # grid_map convention: row index 0 = +x edge, col index 0 = +y edge (see
    # grid_map_util.py docstring), i.e. array[i,j] covers world position
    #   x = pose.x + length_x/2 - i*resolution
    #   y = pose.y + length_y/2 - j*resolution
    info = gm.info
    res = info.resolution
    n_rows, n_cols = elev.shape
    x0 = info.pose.position.x + info.length_x / 2.0
    y0 = info.pose.position.y + info.length_y / 2.0
    ii, jj = np.meshgrid(np.arange(n_rows), np.arange(n_cols), indexing="ij")
    cell_x = x0 - ii * res
    cell_y = y0 - jj * res

    xa, xb, ya, yb = BOX
    finite_mask = np.isfinite(elev)
    out_mask = finite_mask & ((cell_x < xa) | (cell_x > xb) | (cell_y < ya) | (cell_y > yb))
    n_out = int(out_mask.sum())
    lines.append(f"\n## 박스 밖 셀 공간 분포 (100x100 박스 = x[{xa},{xb}] y[{ya},{yb}])\n\n")
    lines.append(f"- 박스 밖에 있는 유효 셀: {n_out} / {n_valid} ({100*n_out/max(n_valid,1):.2f}%)\n")
    if n_out:
        ox = cell_x[out_mask]
        oy = cell_y[out_mask]
        dist = np.maximum.reduce([np.maximum(xa - ox, 0), np.maximum(ox - xb, 0),
                                  np.maximum(ya - oy, 0), np.maximum(oy - yb, 0)])
        lines.append(f"- 박스 경계로부터의 이탈거리: 최소 {dist.min():.3f}m, "
                     f"중앙값 {np.median(dist):.3f}m, 최대 {dist.max():.3f}m\n")
        for edge in (0.5, 5.0, 20.0, 50.0):
            frac = 100.0 * (dist <= edge).sum() / n_out
            lines.append(f"  - 이탈거리 {edge}m 이내: {frac:.1f}%\n")
        worst_i = int(np.argmax(dist))
        lines.append(f"- 최악 이탈점: ({ox[worst_i]:.2f}, {oy[worst_i]:.2f}), "
                     f"고도={elev[out_mask][worst_i]:.3f}, 이탈거리={dist[worst_i]:.3f}m\n")
        np.savez(args.out.replace(".md", "_grid.npz") if args.out else "/tmp/grid.npz",
                elev=elev, cell_x=cell_x, cell_y=cell_y)

    # 박스 기준 커버리지 (root-cause 조사 후 결정: Module A의 grow_to_fit은 건드리지
    # 않고, 채점에서만 100x100 박스를 분모로 쓴다 -- run_results/PROGRESS.md 참조.
    # 이 로직은 이번 100x100 실험 전용 임시 필터다. 500x500으로 확장할 때는
    # 박스 자체가 달라지므로 이 하드코딩된 BOX 상수를 그대로 재사용하면 안 된다.
    in_box_mask = (cell_x >= xa) & (cell_x <= xb) & (cell_y >= ya) & (cell_y <= yb)
    n_valid_in_box = int((finite_mask & in_box_mask).sum())
    box_res = res
    n_box_cells_expected = int(round((xb - xa) / box_res)) * int(round((yb - ya) / box_res))
    box_coverage = 100.0 * n_valid_in_box / n_box_cells_expected
    lines.append(f"\n## 박스 기준 커버리지 (100x100 전용 채점 지표 — Module A 원본은 무수정)\n\n")
    lines.append(f"- 박스 안 유효 셀 / 박스 전체 예상 셀: {n_valid_in_box} / {n_box_cells_expected} "
                 f"= **{box_coverage:.1f}%**\n")
    lines.append(f"- (참고) 원본 GridMap 전체 대비 비-NaN 비율(그리드가 박스보다 커진 것까지 "
                 f"분모에 포함): {coverage:.1f}%\n")
    in_box_valid_vals = elev[finite_mask & in_box_mask]
    if len(in_box_valid_vals):
        lines.append(f"- 박스 안 유효 셀의 고도 범위: {in_box_valid_vals.min():.3f} .. "
                     f"{in_box_valid_vals.max():.3f} m (평균 {in_box_valid_vals.mean():.3f}, "
                     f"중앙값 {np.median(in_box_valid_vals):.3f})\n")

    lines.append(f"## GridMap 기본 통계\n\n")
    lines.append(f"- 격자 크기: {elev.shape[0]} x {elev.shape[1]} (resolution=0.10m 기준)\n")
    lines.append(f"- 측정 커버리지(비-NaN 비율): {coverage:.1f}%\n")
    if n_valid:
        lines.append(f"- 측정 높이 범위: {valid.min():.3f} .. {valid.max():.3f} m "
                     f"(평균 {valid.mean():.3f}, 중앙값 {np.median(valid):.3f})\n")

    # 지표면 모델 대비 기대 범위: 1.202 .. 8.780 (건물 포함). 자기반사/이상치 검사:
    # 드론이 5m AGL로 날았으므로 측정 지표면 값은 이론상 절대 "지표면+5m 근처"
    # (드론 자체 고도)까지 올라갈 이유가 없다. surface+2m 이상을 이상치로 본다
    # (건물 높이 불확실성 + 노이즈 여유).
    xs, ys, terrain, surface, buildings = build_surface_grid(resolution=0.5)
    surf_max = float(surface.max())
    surf_min = float(surface.min())
    anomaly_thresh = surf_max + 2.0
    if n_valid:
        n_anomaly = int((valid > anomaly_thresh).sum())
        lines.append(f"\n## 자기반사/이상치 점검\n\n")
        lines.append(f"- 지표면 모델 기대 범위: {surf_min:.3f} .. {surf_max:.3f} m\n")
        lines.append(f"- 이상치 기준(지표면 최댓값 + 2.0m = {anomaly_thresh:.3f}m) 초과 셀: "
                     f"{n_anomaly} / {n_valid} ({100*n_anomaly/n_valid:.2f}%)\n")
        if n_anomaly:
            over = valid[valid > anomaly_thresh]
            lines.append(f"  - 이상치 값 범위: {over.min():.3f} .. {over.max():.3f} m "
                         f"(자기반사/드론 자체 반사 의심 — 84m 고도 실험 때와 유사한 패턴인지 확인 필요)\n")
        else:
            lines.append("  - 이상치 없음 — 84m 고도 실험에서 우려했던 자기반사 패턴은 "
                         "이번 5m AGL 스캔에서 관측되지 않음.\n")

    report = "".join(lines)
    print(report)
    if args.out:
        with open(args.out, "w") as f:
            f.write(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
