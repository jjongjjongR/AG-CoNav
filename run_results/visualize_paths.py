#!/usr/bin/env python3
import sys
import yaml
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

sys.path.insert(0, "/home/hyunwoo-chae/AG-CoNav-test_main/run_results")
from surface_model import build_surface_grid, load_buildings, BOX  # noqa: E402

CFG_DIR = "/home/hyunwoo-chae/AG-CoNav-test_main/src/agconav_test_worlds/config"

xs, ys, terrain, surface, buildings = build_surface_grid(resolution=0.5)

fig, ax = plt.subplots(figsize=(11, 11), dpi=140)
im = ax.pcolormesh(xs, ys, surface, shading="auto", cmap="terrain")
cbar = fig.colorbar(im, ax=ax, shrink=0.8)
cbar.set_label("surface height = max(terrain, building roof) [m]")

for b in buildings:
    poly = b["poly"]
    ax.add_patch(MplPolygon(poly, closed=True, fill=False, edgecolor="black",
                            linewidth=1.0, alpha=0.8))

colors = {"2m": "#e41a1c", "3m": "#377eb8", "4m": "#4daf4a"}
for tag in ("2m", "3m", "4m"):
    d = yaml.safe_load(open(f"{CFG_DIR}/path_100x100_5m_{tag}.yaml"))
    xs_p = [w["position"]["x"] for w in d["waypoints"]]
    ys_p = [w["position"]["y"] for w in d["waypoints"]]
    ax.plot(xs_p, ys_p, "-", color=colors[tag], linewidth=0.6, alpha=0.75,
            label=f"strip {tag} ({len(xs_p)} wp)")

xa, xb, ya, yb = BOX
ax.set_xlim(xa, xb)
ax.set_ylim(ya, yb)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("5m AGL method-B paths (strip spacing 2m/3m/4m) over surface height + buildings (black outline)")
ax.legend(loc="upper right", fontsize=9)
ax.set_aspect("equal")

out_path = "/home/hyunwoo-chae/AG-CoNav-test_main/run_results/path_5m_visualization.png"
fig.tight_layout()
fig.savefig(out_path)
print("saved", out_path)
