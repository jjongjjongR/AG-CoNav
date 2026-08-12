#!/usr/bin/env python3
"""Carve a small square test world out of a full AG-CoNav Gazebo world.

Loading the whole Seongdong-gu terrain (717.9 x 665.36 m heightmap plus a
686 m wide building mesh) costs far more than a short bag-recording run needs.
This script crops the terrain, the textures and the building mesh down to a
square box and emits a self-contained world directory.

World coordinates are preserved exactly, so the existing spawn arguments in
agconav_sim.launch.py can be reused verbatim: a point that sat at z=5.92 in
the full world still sits at z=5.92 here.

Heightmap pixel <-> world mapping (verified against the ground heights
recorded in agconav_sim.launch.py -- wheel 5.90, leg 5.85):

    col = (x - (pos.x - size.x/2)) / size.x * (width  - 1)
    row = ((pos.y + size.y/2) - y) / size.y * (height - 1)
    elevation = pixel / MAX_PIXEL_IN_IMAGE * size.z + pos.z

i.e. image row 0 is +y (north) and column 0 is -x (west).

Note the denominator: gz-common's ImageHeightmap normalises by the brightest
pixel actually present, *not* by the full 16-bit range. Copying a crop's pixel
values through unchanged therefore inflates its terrain, because the crop's
maximum is far below the source's. Cropping to this 50 m box and keeping
size.z=18.4 turned a 4.20..7.00 m patch into 11.97..18.40 m. So the crop is
re-stretched over the full 16-bit range here and `size.z`/`pos.z` are set to
the patch's true span and floor, which both restores the absolute elevations
and spends all 65535 levels on a ~3 m range instead of ~15% of them.

Usage (defaults reproduce the south-east box used for the GLIM test):

    python3 generate_test_world.py
    python3 generate_test_world.py --corner -162.682 153.831 --dir +x -y --size 50
"""

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path

import numpy as np
from PIL import Image
from lxml import etree

# 16-bit PNG full scale. Gazebo maps pixel/PIXEL_MAX onto <size> z.
PIXEL_MAX = 65535.0

# Heightmap images must be square with a side of 2^n + 1. The side is chosen so
# the crop is never sampled coarser than the source: a 120 m box at 129 px is
# 0.93 m/px against the source's 0.70 m/px, and that downsampling flattens peaks
# by over a metre -- enough that probes dropped at the predicted height missed
# the ground entirely. Upsampling adds no information but loses none either.
HEIGHTMAP_SIDES = [65, 129, 257, 513, 1025]
# Plain textures have no such constraint.
TEXTURE_SIDE = 512

# Buildings are kept or dropped whole, never sliced -- a half-building is a
# hollow shell the LiDAR sees straight through. Which whole ones survive is the
# --buildings choice: "inside" keeps only those that fit entirely within the box,
# "touching" keeps any that reach into it. "touching" is what produced a 50 m
# world dominated by an 80 m tower whose footprint hung far past the terrain
# edge, so "inside" is the default.


def parse_args() -> argparse.Namespace:
    src_root = Path(__file__).resolve().parents[2]
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source",
                   default=str(src_root / "agconav_worlds/worlds/Seongdong_gu/Seongdong_gu.world"),
                   help="source .world file")
    p.add_argument("--output",
                   default=str(Path(__file__).resolve().parents[1] / "worlds/Seongdong_gu_50x50"),
                   help="output world directory")
    p.add_argument("--center", nargs=2, type=float, metavar=("X", "Y"), default=None,
                   help="centre of the box in world coordinates (overrides --corner/--dir)")
    p.add_argument("--corner", nargs=2, type=float, metavar=("X", "Y"),
                   default=[-162.682, 153.831],
                   help="corner of the box in world coordinates (the robot spawn end)")
    p.add_argument("--dir", nargs=2, metavar=("DX", "DY"), default=["+x", "-y"],
                   help="direction the box extends from the corner")
    p.add_argument("--size", type=float, default=50.0, help="box side length [m]")
    p.add_argument("--buildings", choices=["inside", "touching"], default="inside",
                   help="inside: keep only buildings fully within the box (no overhang); "
                        "touching: keep any building that reaches into it")
    p.add_argument("--spawn-model", default="X3",
                   help="include to relocate into the box if the crop leaves it outside")
    p.add_argument("--spawn-inset", type=float, default=10.0,
                   help="how far inside the box corner the relocated model is placed [m]")
    p.add_argument("--world-name", default="Seongdong_gu",
                   help="<world name>. Kept as the source name by default so that "
                        "launch arguments such as world:=Seongdong_gu keep working.")
    return p.parse_args()


def resolve_box(corner: list[float], direction: list[str], size: float):
    """Return (xa, xb, ya, yb) with xa<xb, ya<yb."""
    sx = -1.0 if direction[0].startswith("-") else 1.0
    sy = -1.0 if direction[1].startswith("-") else 1.0
    x_end, y_end = corner[0] + size * sx, corner[1] + size * sy
    xa, xb = sorted((corner[0], x_end))
    ya, yb = sorted((corner[1], y_end))
    return xa, xb, ya, yb


def sample_bilinear(img: np.ndarray, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
    """Bilinear sample of `img` at fractional (rows, cols). Works on 2-D and 3-D."""
    ny, nx = img.shape[0], img.shape[1]
    r0 = np.clip(np.floor(rows).astype(int), 0, ny - 2)
    c0 = np.clip(np.floor(cols).astype(int), 0, nx - 2)
    fr = (rows - r0)[..., None] if img.ndim == 3 else (rows - r0)
    fc = (cols - c0)[..., None] if img.ndim == 3 else (cols - c0)
    return (img[r0, c0] * (1 - fc) * (1 - fr)
            + img[r0, c0 + 1] * fc * (1 - fr)
            + img[r0 + 1, c0] * (1 - fc) * fr
            + img[r0 + 1, c0 + 1] * fc * fr)


def crop_raster(src: Path, dst: Path, box, hm_size, hm_pos, side: int, elevation: bool):
    """Resample the region of `src` covering `box` into a `side` x `side` image.

    For the heightmap (`elevation=True`) the samples are converted to true world
    elevations, re-stretched over the full 16-bit range, and the resulting
    (pos_z, size_z) that reproduces those elevations is returned.
    """
    xa, xb, ya, yb = box
    img = np.array(Image.open(src)).astype(np.float64)
    ny, nx = img.shape[0], img.shape[1]

    x0 = hm_pos[0] - hm_size[0] / 2.0
    y1 = hm_pos[1] + hm_size[1] / 2.0

    xs = np.linspace(xa, xb, side)
    ys = np.linspace(yb, ya, side)          # row 0 is +y
    gx, gy = np.meshgrid(xs, ys)
    cols = (gx - x0) / hm_size[0] * (nx - 1)
    rows = (y1 - gy) / hm_size[1] * (ny - 1)

    out = sample_bilinear(img, rows, cols)
    if not elevation:
        Image.fromarray(np.clip(np.round(out), 0, 255).astype(np.uint8)).save(dst)
        return None

    # Source pixels -> true world elevation, using the same denominator gz uses.
    elev = out / img.max() * hm_size[2] + hm_pos[2]
    z_lo, z_hi = float(elev.min()), float(elev.max())
    span = z_hi - z_lo
    if span <= 0:
        raise SystemExit("the box is perfectly flat; a heightmap cannot be built from it")

    scaled = (elev - z_lo) / span * PIXEL_MAX
    Image.fromarray(np.round(scaled).astype(np.uint16)).save(dst)
    return z_lo, span, elev


def _components(n: int, edges: np.ndarray) -> np.ndarray:
    """Union-find over `edges`, returning a component label per vertex."""
    parent = np.arange(n)

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b in edges:
        ra, rb = find(int(a)), find(int(b))
        if ra != rb:
            parent[ra] = rb
    return np.array([find(i) for i in range(n)])


def crop_dae(src: Path, dst: Path, box, mesh_offset, mode="inside"):
    """Keep buildings whole, selecting them per `mode` (see --buildings)."""
    xa, xb, ya, yb = box
    ox, oy = mesh_offset
    text = src.read_text()

    arrays = re.findall(r"<float_array([^>]*)>(.*?)</float_array>", text, re.S)
    verts = np.fromstring(arrays[0][1], sep=" ").reshape(-1, 3)
    normals = np.fromstring(arrays[1][1], sep=" ").reshape(-1, 3)

    p_match = re.search(r"<p>(.*?)</p>", text, re.S)
    idx = np.fromstring(p_match.group(1), sep=" ", dtype=np.int64).reshape(-1, 2)
    tris = idx.reshape(-1, 3, 2)            # (tri, corner, [vertex, normal])

    # The mesh repeats a position once per face corner, so weld by coordinate
    # before tracing connectivity -- otherwise every triangle is its own island.
    welded, inv = np.unique(np.round(verts, 4), axis=0, return_inverse=True)
    wt = inv[tris[:, :, 0]]
    edges = np.concatenate([wt[:, [0, 1]], wt[:, [1, 2]], wt[:, [2, 0]]])
    label = _components(len(welded), edges)

    # Mesh coordinates are already world-space apart from the model <pose> shift.
    wx, wy = welded[:, 0] + ox, welded[:, 1] + oy
    inside = (wx >= xa) & (wx <= xb) & (wy >= ya) & (wy <= yb)

    if mode == "touching":
        hit = np.unique(label[inside])
    else:
        # A building survives only if every one of its vertices is in the box,
        # so nothing kept can extend past the terrain it stands on.
        touched = np.unique(label[inside])
        hit = np.array([L for L in touched if inside[label == L].all()], dtype=label.dtype)
        dropped = len(touched) - len(hit)
        if dropped:
            print("           경계에 걸친 건물 %d채 제외 (오버행 방지)" % dropped)

    keep = np.isin(label[wt], hit).any(axis=1) if len(hit) else np.zeros(len(wt), bool)
    tris = tris[keep]

    # Re-index so the emitted arrays only carry the vertices still referenced.
    v_used = np.unique(tris[:, :, 0])
    n_used = np.unique(tris[:, :, 1])
    v_map = {int(o): i for i, o in enumerate(v_used)}
    n_map = {int(o): i for i, o in enumerate(n_used)}
    new_v = verts[v_used]
    new_n = normals[n_used]
    flat = []
    for tri in tris:
        for vi, ni in tri:
            flat.append(v_map[int(vi)])
            flat.append(n_map[int(ni)])

    kept_world = welded[np.isin(label, hit)] if len(hit) else welded[:0]
    bbox = ((float(kept_world[:, 0].min() + ox), float(kept_world[:, 0].max() + ox)),
            (float(kept_world[:, 1].min() + oy), float(kept_world[:, 1].max() + oy)),
            (float(kept_world[:, 2].min()), float(kept_world[:, 2].max()))) \
        if len(tris) else ((0, 0), (0, 0), (0, 0))

    def fmt(a):
        return " ".join("%.6g" % v for v in a.reshape(-1))

    out = text
    out = out.replace(arrays[0][1], " " + fmt(new_v) + " ", 1)
    out = out.replace(arrays[1][1], " " + fmt(new_n) + " ", 1)
    out = out.replace(p_match.group(1), " " + " ".join(str(i) for i in flat) + " ", 1)
    out = re.sub(r'(<float_array[^>]*id="verts-array-array"[^>]*count=")\d+',
                 lambda mo: mo.group(1) + str(new_v.size), out)
    out = re.sub(r'(<float_array[^>]*count=")\d+("[^>]*id="verts-array-array")',
                 lambda mo: mo.group(1) + str(new_v.size) + mo.group(2), out)
    out = re.sub(r'(<float_array[^>]*count=")\d+("[^>]*id="normals-array-array")',
                 lambda mo: mo.group(1) + str(new_n.size) + mo.group(2), out)
    out = re.sub(r'(<accessor[^>]*count=")\d+("[^>]*source="#verts-array-array")',
                 lambda mo: mo.group(1) + str(len(new_v)) + mo.group(2), out)
    out = re.sub(r'(<accessor[^>]*count=")\d+("[^>]*source="#normals-array-array")',
                 lambda mo: mo.group(1) + str(len(new_n)) + mo.group(2), out)
    out = re.sub(r'(<triangles[^>]*count=")\d+', lambda mo: mo.group(1) + str(len(tris)), out)
    dst.write_text(out)
    return len(tris), len(new_v), bbox


def text_of(el) -> str:
    return (el.text or "").strip()


def _building_xy(dae: Path, model_pose):
    """World-frame XY of every building vertex, or empty arrays if there are none."""
    if not dae.exists():
        return np.zeros(0), np.zeros(0)
    text = dae.read_text()
    arrays = re.findall(r"<float_array([^>]*)>(.*?)</float_array>", text, re.S)
    if not arrays:
        return np.zeros(0), np.zeros(0)
    v = np.fromstring(arrays[0][1], sep=" ").reshape(-1, 3)
    return v[:, 0] + model_pose[0], v[:, 1] + model_pose[1]


def _find_clearing(box, elev, side, bx, by, edge_margin=12.0, step=2.0):
    """Best open, flat patch inside the box for the robots to start from.

    Ranked on distance to the nearest building first and local flatness second:
    ground robots need somewhere level, and nothing should start inside a wall.
    """
    xa, xb, ya, yb = box

    def ground(x, y):
        j = int(round(min(max((x - xa) / (xb - xa) * (side - 1), 0), side - 1)))
        i = int(round(min(max((yb - y) / (yb - ya) * (side - 1), 0), side - 1)))
        return float(elev[i, j])

    best = None
    for x in np.arange(xa + edge_margin, xb - edge_margin, step):
        for y in np.arange(ya + edge_margin, yb - edge_margin, step):
            clearance = float(np.hypot(bx - x, by - y).min()) if len(bx) else 1e9
            if clearance < 10.0:
                continue
            local = [ground(x + dx, y + dy) for dx in (-5, 0, 5) for dy in (-5, 0, 5)]
            flat = float(np.std(local))
            score = (min(clearance, 25.0), -flat)
            if best is None or score > best[0]:
                best = (score, {"x": float(x), "y": float(y), "z": ground(x, y),
                                "clearance": clearance, "flatness": flat})
    if best is None:
        cx, cy = (xa + xb) / 2, (ya + yb) / 2
        return {"x": cx, "y": cy, "z": ground(cx, cy), "clearance": 0.0, "flatness": 0.0}
    return best[1]


def _camera_pose(box, ground_z):
    """A GUI viewpoint that frames the whole box, looking down at its centre."""
    xa, xb, ya, yb = box
    cx, cy = (xa + xb) / 2.0, (ya + yb) / 2.0
    span = max(xb - xa, yb - ya)
    back = span * 0.9
    up = span * 0.8
    px, py, pz = cx - back, cy - back, ground_z + up
    yaw = np.arctan2(cy - py, cx - px)
    pitch = np.arctan2(pz - ground_z, np.hypot(cx - px, cy - py))
    return "%.2f %.2f %.2f 0 %.4f %.4f" % (px, py, pz, pitch, yaw)


def main() -> int:
    args = parse_args()
    src_world = Path(args.source).resolve()
    out_dir = Path(args.output).resolve()
    src_mesh = src_world.parent / "mesh"
    out_mesh = out_dir / "mesh"
    out_mesh.mkdir(parents=True, exist_ok=True)

    if args.center:
        h = args.size / 2.0
        box = (args.center[0] - h, args.center[0] + h,
               args.center[1] - h, args.center[1] + h)
    else:
        box = resolve_box(args.corner, args.dir, args.size)
    xa, xb, ya, yb = box
    cx, cy = (xa + xb) / 2.0, (ya + yb) / 2.0
    print("box  x[%.3f, %.3f]  y[%.3f, %.3f]  center (%.3f, %.3f)" % (xa, xb, ya, yb, cx, cy))

    tree = etree.parse(str(src_world))
    root = tree.getroot()
    world = root.find("world")
    world.set("name", args.world_name)

    # --- heightmap geometry: read the source extent before rewriting it -------
    hm = world.find(".//heightmap")
    hm_size = [float(v) for v in text_of(hm.find("size")).split()]
    vis_hm = world.find(".//visual/geometry/heightmap")
    hm_pos = [float(v) for v in text_of(vis_hm.find("pos")).split()]
    print("source heightmap size=%s pos=%s" % (hm_size, hm_pos))

    # --- rasters -------------------------------------------------------------
    src_hm = np.array(Image.open(src_mesh / "height_map.png"))
    src_res = max(hm_size[0] / (src_hm.shape[1] - 1), hm_size[1] / (src_hm.shape[0] - 1))
    needed = args.size / src_res + 1
    side = next((s for s in HEIGHTMAP_SIDES if s >= needed), HEIGHTMAP_SIDES[-1])
    print("heightmap 격자: 원본 %.3f m/px → %d x %d (%.3f m/px)"
          % (src_res, side, side, args.size / (side - 1)))

    z_lo, z_span, elev = crop_raster(src_mesh / "height_map.png", out_mesh / "height_map.png",
                                     box, hm_size, hm_pos, side, elevation=True)
    for name in ("aerial.png", "normal_map.png"):
        crop_raster(src_mesh / name, out_mesh / name,
                    box, hm_size, hm_pos, TEXTURE_SIDE, elevation=False)
    print("heightmap %dx%d  elevation %.3f .. %.3f m (relief %.3f)  ->  size.z=%.4f pos.z=%.4f"
          % (side, side, z_lo, z_lo + z_span, z_span, z_span, z_lo))

    # --- building mesh -------------------------------------------------------
    bld_model = world.find(".//model[@name='Seongdong_gu_buildings']")
    bld_pose = [float(v) for v in text_of(bld_model.find("pose")).split()]
    n_tris, n_verts, bld_bbox = crop_dae(src_mesh / "buildings.dae", out_mesh / "buildings.dae",
                                         box, (bld_pose[0], bld_pose[1]), args.buildings)
    print("buildings: %d triangles / %d vertices kept" % (n_tris, n_verts))
    if not n_tris:
        # An empty COLLADA is not something Gazebo should be asked to load.
        # The buildings model is nested inside the terrain model, not the world.
        bld_model.getparent().remove(bld_model)
        (out_mesh / "buildings.dae").unlink()
        print("           건물이 없어 buildings 모델을 통째로 제거")
    if n_tris:
        # Buildings are kept whole, so one that only clips the box drags its
        # whole footprint in with it and ends up hanging over the terrain edge.
        # A 50 m box that caught the corner of an 80 m tower looked, fairly,
        # broken. Say how far the geometry runs past the ground it sits on.
        overhang = max(xa - bld_bbox[0][0], bld_bbox[0][1] - xb,
                       ya - bld_bbox[1][0], bld_bbox[1][1] - yb, 0.0)
        print("           bbox x[%.1f, %.1f] y[%.1f, %.1f] 최고 %.1fm"
              % (bld_bbox[0][0], bld_bbox[0][1], bld_bbox[1][0], bld_bbox[1][1],
                 bld_bbox[2][1]))
        if overhang > 1.0:
            print("  [경고] 건물이 지형 밖으로 %.1fm 뻗어 있습니다 — 허공에 뜬 것처럼 보입니다."
                  % overhang)

    # --- rewrite the heightmap entries --------------------------------------
    new_size = "%.4f %.4f %.6f" % (args.size, args.size, z_span)
    col = world.find(".//collision[@name='collision']")
    col.find("pose").text = "%.4f %.4f %.6f 0 0 0" % (cx, cy, z_lo)
    for node in world.iterfind(".//heightmap"):
        node.find("size").text = new_size
    # DART ignores <pos> for collision heightmaps; OGRE2 ignores link pose for visuals.
    col.find(".//heightmap/pos").text = "0 0 0"
    vis_hm.find("pos").text = "%.4f %.4f %.6f" % (cx, cy, z_lo)
    tex = vis_hm.find("texture/size")
    if tex is not None:
        tex.text = "%.4f" % args.size

    # --- drop every include that sits outside the box ------------------------
    # The drone is the exception: cropping somewhere it does not happen to stand
    # would otherwise produce a world with nothing to fly. It gets moved to a
    # corner of the box instead, keeping its original clearance above ground.
    def ground_at(x, y):
        j = (x - xa) / (xb - xa) * (side - 1)
        i = (yb - y) / (yb - ya) * (side - 1)
        i = int(round(min(max(i, 0), side - 1)))
        j = int(round(min(max(j, 0), side - 1)))
        return float(elev[i, j])

    src_max = np.array(Image.open(src_mesh / "height_map.png")).astype(np.float64).max()

    def source_ground(x, y):
        img = np.array(Image.open(src_mesh / "height_map.png")).astype(np.float64)
        ny, nx = img.shape
        col = (x - (hm_pos[0] - hm_size[0] / 2)) / hm_size[0] * (nx - 1)
        row = ((hm_pos[1] + hm_size[1] / 2) - y) / hm_size[1] * (ny - 1)
        v = sample_bilinear(img, np.array([row]), np.array([col]))[0]
        return float(v / src_max * hm_size[2] + hm_pos[2])

    # --- pick a spawn clearing and point the GUI camera at the box -----------
    # The source world's spawn point and camera both sit wherever the robots
    # happened to be in the full map; after a crop they are usually nowhere
    # near the remaining terrain. Both get recomputed for this box.
    bx, by = _building_xy(src_mesh / "buildings.dae", bld_pose)
    clearing = _find_clearing(box, elev, side, bx, by)
    print("스폰 개활지: (%.2f, %.2f) 지면 %.3fm  건물이격 %.1fm  평탄도 std %.3f"
          % (clearing["x"], clearing["y"], clearing["z"],
             clearing["clearance"], clearing["flatness"]))

    cam = world.find(".//gui//camera_pose")
    if cam is not None:
        cam.text = _camera_pose(box, float(np.median(elev)))
        print("GUI 카메라: %s" % cam.text)

    # The three robots keep the relative layout and ground clearances they have
    # in agconav_sim.launch.py -- only the clearing they stand in moves.
    SPAWN_LAYOUT = {                    # dx, dy from the clearing, clearance, yaw
        "wheel": (0.000, 0.000, 0.250, -0.4349),
        "leg":   (0.811, 3.293, 0.300, -0.4613),
        "drone": (3.202, 0.642, 0.156, 0.0000),
    }
    spawns = {}
    for who, (dx, dy, clr, yaw) in SPAWN_LAYOUT.items():
        sx, sy = clearing["x"] + dx, clearing["y"] + dy
        spawns[who] = (sx, sy, ground_at(sx, sy) + clr, yaw)

    kept, dropped, relocated = [], 0, None
    for inc in list(world.iterfind("include")):
        pose = inc.find("pose")
        name = inc.find("name")
        label = text_of(name) if name is not None else "?"
        if pose is None:
            continue
        v = [float(t) for t in text_of(pose).split()]
        if label == args.spawn_model:
            sx, sy, sz, _ = spawns["drone"]
            pose.text = "%.4f %.4f %.4f %s" % (
                sx, sy, sz, " ".join("%.4f" % t for t in v[3:6]) or "0 0 0")
            relocated = (sx, sy, sz)
            kept.append(label + " (이동)")
        elif xa <= v[0] <= xb and ya <= v[1] <= yb:
            kept.append(label)
        else:
            world.remove(inc)
            dropped += 1
    print("includes kept: %s   (dropped %d)" % (", ".join(kept) or "(없음)", dropped))
    if relocated:
        print("  %s 스폰 → (%.3f, %.3f, %.3f)" % (args.spawn_model, *relocated))
    print("\n  agconav_sim.launch.py 스폰 인자 (이 월드용):")
    for who in ("wheel", "leg"):
        sx, sy, sz, yaw = spawns[who]
        print("    %s_x:=%.4f %s_y:=%.4f %s_z:=%.4f %s_yaw:=%.4f"
              % (who, sx, who, sy, who, sz, who, yaw))

    out_world = out_dir / (out_dir.name + ".world")
    tree.write(str(out_world), pretty_print=True, xml_declaration=True, encoding="utf-8")

    cfg = src_world.parent / "model.config"
    if cfg.exists():
        text = cfg.read_text().replace("<name>Seongdong_gu</name>",
                                       "<name>%s</name>" % out_dir.name)
        (out_dir / "model.config").write_text(text)

    print("\nwrote %s" % out_world)
    for f in sorted(out_dir.rglob("*")):
        if f.is_file():
            print("  %-28s %8.1f KB" % (f.relative_to(out_dir), f.stat().st_size / 1024))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
