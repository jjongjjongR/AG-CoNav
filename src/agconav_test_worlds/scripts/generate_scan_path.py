#!/usr/bin/env python3
"""Generate the scan path for a test world using agconav_drone's own generator.

This is a wrapper, not a reimplementation. It loads
agconav_drone/scripts/generate_path.py and only redirects its WORLD_FILE and
OUTPUT_FILE, so altitude, vertical FOV, overlap ratio, the strip-spacing
formula and the lawnmower construction all stay in that one file. The scan
range changes because the world's heightmap extent changes; nothing else does.

The result is written into this package's config/ so the full-world
agconav_drone/config/path.yaml is left untouched, and it is replayed by
agconav_drone's unmodified drone_path_player -- which is what keeps a run here
behaving exactly as it will when pointed back at the full world.
"""

import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.dirname(PKG)

GENERATOR = os.path.join(SRC, "agconav_drone", "scripts", "generate_path.py")
WORLD = os.path.join(PKG, "worlds", "Seongdong_gu_100x100", "Seongdong_gu_100x100.world")
OUTPUT = os.path.join(PKG, "config", "path_100x100.yaml")


def main() -> int:
    world = sys.argv[1] if len(sys.argv) > 1 else WORLD
    output = sys.argv[2] if len(sys.argv) > 2 else OUTPUT

    spec = importlib.util.spec_from_file_location("agconav_generate_path", GENERATOR)
    gp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gp)

    gp.WORLD_FILE = world
    gp.OUTPUT_FILE = output
    os.makedirs(os.path.dirname(output), exist_ok=True)

    print("[wrapper] 비행 파라미터는 agconav_drone/scripts/generate_path.py 값을 그대로 사용")
    print("[wrapper]   ALTITUDE=%.1fm  VERTICAL_FOV=%.1f도  OVERLAP=%.2f"
          % (gp.ALTITUDE, gp.VERTICAL_FOV_DEG, gp.OVERLAP_RATIO))
    return gp.main() or 0


if __name__ == "__main__":
    raise SystemExit(main())
