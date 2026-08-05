#!/usr/bin/env python3
"""
generate_path.py

Seongdong_gu.world의 heightmap 타일 전체(건물 위치와 무관하게 지형 타일
전체 사각형)를 lawnmower(지그재그) 패턴으로 빈틈없이 스캔하는 waypoint
목록을 생성해 path.yaml로 저장한다.

스캔 범위는 world 파일의 <heightmap><size>와 그 pose(XY 오프셋)를 직접
파싱해서 구한다 (하드코딩 금지).

STRIP_SPACING은 드론 LiDAR(Ouster OS1-32) 스펙과 비행 고도로부터
공식으로 계산한다 (README/팀 확정값: ALTITUDE=84m, OVERLAP_RATIO=0.5):

    swath_width   = 2 * ALTITUDE * tan(VERTICAL_FOV_DEG / 2)
    STRIP_SPACING = swath_width * (1 - OVERLAP_RATIO)
"""

import math
import os
import xml.etree.ElementTree as ET

import yaml

# ===== 센서/비행 파라미터 (팀 확정값) =====
ALTITUDE = 84.0              # 비행 고도 z (m)
VERTICAL_FOV_DEG = 42.4      # Ouster OS1-32 수직 FOV (도)
OVERLAP_RATIO = 0.5          # 스트립 간 겹침 비율 (팀 논의로 확정, 50%)

# swath_width, STRIP_SPACING은 위 파라미터로부터 실제로 계산한다.
# (고도나 센서가 바뀌면 이 값들도 자동으로 재계산됨)
_HALF_FOV_RAD = math.radians(VERTICAL_FOV_DEG / 2.0)
SWATH_WIDTH = 2.0 * ALTITUDE * math.tan(_HALF_FOV_RAD)
STRIP_SPACING = SWATH_WIDTH * (1.0 - OVERLAP_RATIO)

FRAME_ID = "map"

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORLD_FILE = os.path.join(
    _SCRIPT_DIR, "..", "..", "agconav_worlds",
    "worlds", "Seongdong_gu", "Seongdong_gu.world",
)
OUTPUT_FILE = os.path.join(_SCRIPT_DIR, "..", "config", "path.yaml")


def parse_heightmap_extent(world_file):
    """world 파일에서 heightmap의 <size>(x,y)와 world-frame XY 오프셋을 읽는다.

    오프셋은 heightmap을 담은 model의 <pose>와, 그 안 collision의 <pose>를
    합산해서 구한다 (Seongdong_gu.world는 둘 다 회전이 0이라 XY 단순 합산으로
    충분하다). 회전이 있는 pose가 발견되면 경고를 출력한다 - 이 스크립트는
    회전이 있는 체인은 지원하지 않는다.
    """
    tree = ET.parse(world_file)
    root = tree.getroot()

    for model in root.iter("model"):
        heightmap = model.find(".//link/collision/geometry/heightmap")
        if heightmap is None:
            continue

        size_text = heightmap.findtext("size")
        if not size_text:
            continue
        size_vals = [float(v) for v in size_text.split()]
        size_x, size_y = size_vals[0], size_vals[1]

        model_pose = _parse_pose(model.findtext("pose"))
        collision = model.find(".//link/collision")
        collision_pose = _parse_pose(collision.findtext("pose"))

        for label, pose in (("model", model_pose), ("collision", collision_pose)):
            if any(abs(v) > 1e-6 for v in pose[3:6]):
                print(
                    f"[경고] {label} pose에 회전(roll/pitch/yaw)이 있어 "
                    f"XY 단순 합산이 부정확할 수 있습니다: {pose}"
                )

        offset_x = model_pose[0] + collision_pose[0]
        offset_y = model_pose[1] + collision_pose[1]

        return {
            "model_name": model.get("name"),
            "size_x": size_x,
            "size_y": size_y,
            "offset_x": offset_x,
            "offset_y": offset_y,
        }

    raise RuntimeError(f"{world_file}에서 <heightmap>을 찾지 못했습니다.")


def _parse_pose(pose_text):
    if not pose_text:
        return [0.0] * 6
    vals = [float(v) for v in pose_text.split()]
    while len(vals) < 6:
        vals.append(0.0)
    return vals


def yaw_to_quaternion(yaw_rad):
    """yaw(라디안, z축 회전)만 있는 수평 자세를 quaternion으로 변환"""
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw_rad / 2.0),
        "w": math.cos(yaw_rad / 2.0),
    }


def generate_lawnmower_waypoints(x_min, x_max, y_min, y_max, nominal_spacing):
    """지그재그(lawnmower) waypoint 목록을 생성한다.

    줄 수(strip count)를 올림해서 실제 줄 간격을 nominal_spacing 이하로
    맞춘다 - 그래야 y_min과 y_max를 정확히 포함하면서 마지막 줄에서
    스캔이 빠지는 틈(gap)이 생기지 않는다.
    """
    y_range = y_max - y_min
    if y_range <= 0:
        num_strips = 1
    else:
        num_strips = max(1, math.ceil(y_range / nominal_spacing) + 1)

    if num_strips == 1:
        actual_spacing = 0.0
        y_positions = [y_min]
    else:
        actual_spacing = y_range / (num_strips - 1)
        y_positions = [y_min + i * actual_spacing for i in range(num_strips)]

    waypoints = []
    going_right = True  # 첫 줄은 x_min -> x_max 방향

    for y in y_positions:
        if going_right:
            x_start, x_end = x_min, x_max
            yaw = 0.0  # +x 방향을 바라봄
        else:
            x_start, x_end = x_max, x_min
            yaw = math.pi  # -x 방향을 바라봄

        orientation = yaw_to_quaternion(yaw)

        waypoints.append({
            "position": {"x": round(x_start, 3), "y": round(y, 3), "z": ALTITUDE},
            "orientation": orientation,
        })
        waypoints.append({
            "position": {"x": round(x_end, 3), "y": round(y, 3), "z": ALTITUDE},
            "orientation": orientation,
        })

        going_right = not going_right

    return waypoints, num_strips, actual_spacing


def main():
    print(f"[world 파싱] 파일: {os.path.normpath(WORLD_FILE)}")
    extent = parse_heightmap_extent(WORLD_FILE)
    print(
        f"[world 파싱] model='{extent['model_name']}' "
        f"heightmap size=({extent['size_x']}, {extent['size_y']})m "
        f"offset=({extent['offset_x']}, {extent['offset_y']})m"
    )

    x_min = extent["offset_x"] - extent["size_x"] / 2.0
    x_max = extent["offset_x"] + extent["size_x"] / 2.0
    y_min = extent["offset_y"] - extent["size_y"] / 2.0
    y_max = extent["offset_y"] + extent["size_y"] / 2.0

    print(
        f"[스캔 범위] X [{x_min:.2f}, {x_max:.2f}]m, "
        f"Y [{y_min:.2f}, {y_max:.2f}]m (건물 위치와 무관하게 지형 타일 전체)"
    )
    print(
        f"[STRIP_SPACING 계산] ALTITUDE={ALTITUDE}m, "
        f"VERTICAL_FOV_DEG={VERTICAL_FOV_DEG}도, OVERLAP_RATIO={OVERLAP_RATIO}"
    )
    print(f"  swath_width   = 2 * {ALTITUDE} * tan({VERTICAL_FOV_DEG / 2}도) = {SWATH_WIDTH:.3f}m")
    print(f"  STRIP_SPACING = {SWATH_WIDTH:.3f} * (1 - {OVERLAP_RATIO}) = {STRIP_SPACING:.3f}m")

    waypoints, num_strips, actual_spacing = generate_lawnmower_waypoints(
        x_min, x_max, y_min, y_max, STRIP_SPACING
    )
    if abs(actual_spacing - STRIP_SPACING) > 1e-6:
        print(
            f"[줄 간격 보정] 빈틈없이 y_max까지 덮기 위해 실제 간격을 "
            f"{actual_spacing:.3f}m로 보정 (계산값 {STRIP_SPACING:.3f}m 이하로 촘촘해짐)"
        )

    data = {
        "frame_id": FRAME_ID,
        "altitude": ALTITUDE,
        "strip_spacing": round(STRIP_SPACING, 3),
        "num_waypoints": len(waypoints),
        "waypoints": waypoints,
    }

    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    print(f"[결과] 총 줄 수: {num_strips}, 총 waypoint 수: {len(waypoints)}")
    print(
        f"[결과] 커버 범위: X {x_max - x_min:.2f}m x Y {y_max - y_min:.2f}m, "
        f"고도 {ALTITUDE}m"
    )
    print(f"[결과] 저장 완료: {os.path.normpath(OUTPUT_FILE)}")


if __name__ == "__main__":
    main()
