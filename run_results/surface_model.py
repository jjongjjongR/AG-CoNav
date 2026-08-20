#!/usr/bin/env python3
"""5m AGL 방법B 경로 생성용 지표면 높이 모델.

지표면 높이(x,y) = max(지형 높이, 그 지점을 덮는 건물이 있으면 그 건물의 지붕 Z).

- 지형: height_map.png를 world 파일의 <heightmap><size>/<pos>로 디코딩(쌍선형 보간).
  디코딩 공식은 generate_test_world.py 문서화 그대로:
      col = (x - (pos.x - size.x/2)) / size.x * (width-1)
      row = ((pos.y + size.y/2) - y) / size.y * (height-1)
      elevation = pixel / img.max() * size.z + pos.z
- 건물: buildings.dae를 정점 welding + union-find로 개별 건물(연결요소)로 분리한
  뒤(generate_test_world.py의 crop_dae()와 동일 로직 재사용), 각 건물의 (x,y) 볼록껍질
  polygon과 지붕 Z(z1, = 바닥+3.0m 균일)를 구한다.
  포함 판정은 볼록껍질(convex hull) 기준 -- 실제 건물 풋프린트가 오목할 수 있으므로
  볼록껍질은 실제보다 넓게 잡힐 수 있는데, 이는 "보수적으로(더 높게) 추정" 요구사항과
  방향이 맞다: 오탐(건물 아닌데 건물로 판정)은 있어도 미탐(건물인데 못 잡음)은 없다.
- 격자 캐시: 0.5m 간격으로 100x100 박스 전체를 미리 계산해 npz로 저장.
"""
from __future__ import annotations

import re
import numpy as np
from PIL import Image
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
WORLD_DIR = _REPO_ROOT / "src/agconav_test_worlds/worlds/Seongdong_gu_100x100"
HM_PNG = WORLD_DIR / "mesh" / "height_map.png"
DAE = WORLD_DIR / "mesh" / "buildings.dae"

# world 파일에서 실측(1단계 조사 결과와 동일)
HM_SIZE = (100.0000, 100.0000, 5.249597)
HM_POS = (11.4000, -116.1000, 1.202132)
BLD_POSE = (11.39, 1.61, 0.0)

BOX = (-38.60, 61.40, -166.10, -66.10)  # xa, xb, ya, yb


def load_terrain():
    img = np.array(Image.open(HM_PNG)).astype(np.float64)
    return img


def terrain_elevation(xs: np.ndarray, ys: np.ndarray, img: np.ndarray) -> np.ndarray:
    """쌍선형 보간으로 임의의 (x,y) 배열에 대한 지형 고도를 반환."""
    ny, nx = img.shape
    x0 = HM_POS[0] - HM_SIZE[0] / 2.0
    y1 = HM_POS[1] + HM_SIZE[1] / 2.0
    cols = (xs - x0) / HM_SIZE[0] * (nx - 1)
    rows = (y1 - ys) / HM_SIZE[1] * (ny - 1)
    cols = np.clip(cols, 0, nx - 1)
    rows = np.clip(rows, 0, ny - 1)
    c0 = np.clip(np.floor(cols).astype(int), 0, nx - 2)
    r0 = np.clip(np.floor(rows).astype(int), 0, ny - 2)
    fc = cols - c0
    fr = rows - r0
    px = (img[r0, c0] * (1 - fc) * (1 - fr)
          + img[r0, c0 + 1] * fc * (1 - fr)
          + img[r0 + 1, c0] * (1 - fc) * fr
          + img[r0 + 1, c0 + 1] * fc * fr)
    return px / img.max() * HM_SIZE[2] + HM_POS[2]


def _components(n, edges):
    parent = np.arange(n)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b in edges:
        ra, rb = find(int(a)), find(int(b))
        if ra != rb:
            parent[ra] = rb
    return np.array([find(i) for i in range(n)])


def load_buildings():
    """건물별 (convex hull xy vertices, roof_z) 리스트를 반환."""
    text = DAE.read_text()
    arrays = re.findall(r"<float_array([^>]*)>(.*?)</float_array>", text, re.S)
    verts = np.fromstring(arrays[0][1], sep=" ").reshape(-1, 3)
    p_match = re.search(r"<p>(.*?)</p>", text, re.S)
    idx = np.fromstring(p_match.group(1), sep=" ", dtype=np.int64).reshape(-1, 2)
    tris = idx.reshape(-1, 3, 2)

    welded, inv = np.unique(np.round(verts, 4), axis=0, return_inverse=True)
    wt = inv[tris[:, :, 0]]
    edges = np.concatenate([wt[:, [0, 1]], wt[:, [1, 2]], wt[:, [2, 0]]])
    label = _components(len(welded), edges)

    ox, oy = BLD_POSE[0], BLD_POSE[1]
    wx = welded[:, 0] + ox
    wy = welded[:, 1] + oy
    wz = welded[:, 2]

    from scipy.spatial import ConvexHull

    buildings = []
    for L in np.unique(label):
        m = label == L
        if m.sum() < 3:
            continue
        pts = np.column_stack([wx[m], wy[m]])
        roof_z = float(wz[m].max())
        try:
            hull = ConvexHull(pts)
            poly = pts[hull.vertices]
        except Exception:
            poly = pts
        buildings.append({"poly": poly, "roof_z": roof_z,
                           "bbox": (pts[:, 0].min(), pts[:, 0].max(),
                                    pts[:, 1].min(), pts[:, 1].max())})
    return buildings


def _point_in_poly(px, py, poly):
    """표준 ray casting. 배열 px,py에 대해 벡터화."""
    n = len(poly)
    inside = np.zeros(px.shape, dtype=bool)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cond = ((yi > py) != (yj > py)) & (
            px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi)
        inside ^= cond
        j = i
    return inside


def build_surface_grid(resolution=0.5, cache_path=None):
    xa, xb, ya, yb = BOX
    xs = np.arange(xa, xb + 1e-9, resolution)
    ys = np.arange(ya, yb + 1e-9, resolution)
    gx, gy = np.meshgrid(xs, ys)  # shape (len(ys), len(xs))

    img = load_terrain()
    terrain = terrain_elevation(gx, gy, img)

    surface = terrain.copy()
    buildings = load_buildings()
    for b in buildings:
        x0, x1, y0, y1 = b["bbox"]
        # bbox로 먼저 걸러서 point-in-polygon 비용을 줄인다.
        mask_box = (gx >= x0 - 0.5) & (gx <= x1 + 0.5) & (gy >= y0 - 0.5) & (gy <= y1 + 0.5)
        if not mask_box.any():
            continue
        idxs = np.where(mask_box)
        inside = _point_in_poly(gx[idxs], gy[idxs], b["poly"])
        rows = idxs[0][inside]
        cols = idxs[1][inside]
        surface[rows, cols] = np.maximum(surface[rows, cols], b["roof_z"])

    if cache_path:
        np.savez(cache_path, xs=xs, ys=ys, terrain=terrain, surface=surface)
    return xs, ys, terrain, surface, buildings


class SurfaceModel:
    """격자 캐시에서 최근접(및 필요시 보간) 조회를 제공하는 헬퍼.

    보수적(과소평가 금지) 요구사항 때문에, 임의 (x,y) 조회는 그 지점을 감싸는
    2x2 격자 셀 중 **최댓값**을 취한다(쌍선형 보간이 아니라 max) -- 그래야 건물
    가장자리에서 인접 지형 셀과 섞여 건물 높이를 낮잡아 보간하는 일이 없다.
    """

    def __init__(self, xs, ys, surface):
        self.xs = xs
        self.ys = ys
        self.surface = surface
        self.res = xs[1] - xs[0]

    def height_at(self, x, y):
        xs, ys, surf = self.xs, self.ys, self.surface
        xa, xb, ya, yb = xs[0], xs[-1], ys[0], ys[-1]
        x = min(max(x, xa), xb)
        y = min(max(y, ya), yb)
        ci = (x - xa) / self.res
        ri = (y - ya) / self.res
        c0 = int(np.clip(np.floor(ci), 0, len(xs) - 1))
        c1 = int(np.clip(c0 + 1, 0, len(xs) - 1))
        r0 = int(np.clip(np.floor(ri), 0, len(ys) - 1))
        r1 = int(np.clip(r0 + 1, 0, len(ys) - 1))
        return float(max(surf[r0, c0], surf[r0, c1], surf[r1, c0], surf[r1, c1]))

    def height_at_array(self, xarr, yarr):
        """벡터화된 조회. 각 점을 감싸는 2x2 셀 중 최댓값(보수적)."""
        xs, ys, surf = self.xs, self.ys, self.surface
        xa, xb, ya, yb = xs[0], xs[-1], ys[0], ys[-1]
        xarr = np.clip(np.asarray(xarr, dtype=float), xa, xb)
        yarr = np.clip(np.asarray(yarr, dtype=float), ya, yb)
        ci = (xarr - xa) / self.res
        ri = (yarr - ya) / self.res
        c0 = np.clip(np.floor(ci).astype(int), 0, len(xs) - 1)
        c1 = np.clip(c0 + 1, 0, len(xs) - 1)
        r0 = np.clip(np.floor(ri).astype(int), 0, len(ys) - 1)
        r1 = np.clip(r0 + 1, 0, len(ys) - 1)
        vals = np.stack([surf[r0, c0], surf[r0, c1], surf[r1, c0], surf[r1, c1]], axis=0)
        return vals.max(axis=0)


if __name__ == "__main__":
    xs, ys, terrain, surface, buildings = build_surface_grid(
        resolution=0.5, cache_path=str(_REPO_ROOT / "run_results/surface_cache.npz"))
    print(f"건물 수: {len(buildings)}")
    print(f"지형 고도 범위: {terrain.min():.3f} .. {terrain.max():.3f}")
    print(f"지표면(건물포함) 고도 범위: {surface.min():.3f} .. {surface.max():.3f}")
    covered = (surface > terrain + 0.01).sum()
    print(f"건물이 지형보다 높게 잡힌 셀 비율: {100*covered/surface.size:.1f}% "
          f"({covered}/{surface.size} @ 0.5m grid)")
