#!/usr/bin/env python3
"""capture_nav.py를 기반으로, /wheel/nav_map·/leg/nav_map을 GT 통과가능 마스크
(gt_traversable.py)와 대조해 wheel FN%/leg FN%까지 계산한다.

FN% 정의(사용자 지시): GT로 "실제로 통과 가능"한 셀 중, 우리 파이프라인이 그 셀을
"막힘"(OCCUPIED=100)으로 판정한 비율. UNKNOWN(-1, 미측정)은 "막힘 판정"이 아니므로
FN에 넣지 않고 별도로 보고한다(측정 누락은 다른 실패 모드).

결과를 JSON으로 저장해 다음 단계(최종 비교표 조립)에서 그대로 읽을 수 있게 한다.

    python3 capture_nav_fn.py <태그> <결과.json> [timeout]
"""
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gt_traversable import build as build_gt  # noqa: E402

WANT = {'/wheel/nav_map', '/leg/nav_map', 'elev', 'feat'}
LIMITS = {'slope': (20.0, 30.0), 'step': (0.08, 0.15)}


def occgrid_to_array(m):
    """(rows, cols) int16 배열 + 각 셀의 world (x,y) 중심좌표(1D xs/ys)."""
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    d = np.array(m.data, dtype=np.int16).reshape(h, w)  # row-major, row0=y=oy
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    return d, xs, ys


def fn_rate(occ_d, occ_xs, occ_ys, gt_mask, gt_xs, gt_ys):
    """gt_mask(gt_ys x gt_xs shape)가 True인 셀 각각에 대해, occ 격자에서 가장
    가까운 셀의 판정을 찾아 OCCUPIED(>=100)면 FN, UNKNOWN(<0)이면 unknown으로 센다."""
    gt_yi, gt_xi = np.where(gt_mask)
    gx = gt_xs[gt_xi]
    gy = gt_ys[gt_yi]

    ox_res = occ_xs[1] - occ_xs[0] if len(occ_xs) > 1 else 0.1
    oy_res = occ_ys[1] - occ_ys[0] if len(occ_ys) > 1 else 0.1
    ci = np.clip(np.round((gx - occ_xs[0]) / ox_res).astype(int), 0, len(occ_xs) - 1)
    ri = np.clip(np.round((gy - occ_ys[0]) / oy_res).astype(int), 0, len(occ_ys) - 1)
    verdict = occ_d[ri, ci]

    n_gt = len(gx)
    n_occupied = int((verdict >= 100).sum())
    n_unknown = int((verdict < 0).sum())
    n_free = int((verdict == 0).sum())
    return {
        "n_gt_traversable": int(n_gt),
        "n_pipeline_occupied_FN": n_occupied,
        "n_pipeline_unknown": n_unknown,
        "n_pipeline_free_correct": n_free,
        "FN_percent": 100.0 * n_occupied / n_gt if n_gt else float("nan"),
        "unknown_percent": 100.0 * n_unknown / n_gt if n_gt else float("nan"),
    }


class Capture(Node):
    def __init__(self, tag, out_path, timeout):
        super().__init__('capture_nav_fn')
        self.tag = tag
        self.out_path = out_path
        self.seen = set()
        self.result = {"tag": tag}
        self.gt = build_gt(0.10)
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        self._grid_msgs = {}
        for t in ('/wheel/nav_map', '/leg/nav_map'):
            self.create_subscription(OccupancyGrid, t,
                                     lambda m, t=t: self.on_grid(t, m), q)
        self.create_subscription(GridMap, '/drone/elevation_map', self.on_elev, q)
        self.create_subscription(GridMap, '/terrain/features', self.on_feat, q)
        self.create_timer(timeout, self.finish)

    def on_elev(self, m):
        if 'elev' in self.seen:
            return
        self.seen.add('elev')
        d = np.array(m.data[m.layers.index('elevation')].data, dtype=np.float64)
        ok = np.isfinite(d)
        self.result["elevation_map"] = {
            "size_m": [m.info.length_x, m.info.length_y], "resolution": m.info.resolution,
            "n_cells": int(d.size), "n_valid": int(ok.sum()),
            "coverage_percent": 100 * float(ok.mean()) if d.size else float("nan"),
            "height_min": float(d[ok].min()) if ok.any() else None,
            "height_max": float(d[ok].max()) if ok.any() else None,
        }
        self.maybe_done()

    def on_feat(self, m):
        if 'feat' in self.seen:
            return
        self.seen.add('feat')
        feat = {}
        for name, (wheel, leg) in LIMITS.items():
            if name not in m.layers:
                continue
            d = np.array(m.data[m.layers.index(name)].data, dtype=np.float64)
            v = d[np.isfinite(d)]
            if not v.size:
                continue
            feat[name] = {
                "median": float(np.median(v)), "p95": float(np.percentile(v, 95)),
                "max": float(v.max()),
                "wheel_exceed_percent": 100 * float((v > wheel).mean()),
                "leg_exceed_percent": 100 * float((v > leg).mean()),
            }
        self.result["features"] = feat
        self.maybe_done()

    def on_grid(self, topic, m):
        if topic in self.seen:
            return
        self.seen.add(topic)
        self._grid_msgs[topic] = m
        d = np.array(m.data, dtype=np.int16)
        unknown, free, blocked = (d < 0).sum(), (d == 0).sum(), (d >= 100).sum()
        known = free + blocked
        self.result[topic] = {
            "width": m.info.width, "height": m.info.height, "resolution": m.info.resolution,
            "unknown_percent": 100 * float(unknown) / d.size,
            "free_percent": 100 * float(free) / d.size,
            "blocked_percent": 100 * float(blocked) / d.size,
            "free_of_known_percent": (100 * float(free) / known) if known else None,
        }
        self.maybe_done()

    def maybe_done(self):
        if WANT <= self.seen:
            self.finish()

    def finish(self):
        missing = WANT - self.seen
        if missing:
            self.result["missing"] = sorted(missing)
            print('[%s] 수신 실패: %s' % (self.tag, ', '.join(sorted(missing))))
        else:
            for robot, topic, gtkey in (("wheel", "/wheel/nav_map", "wheel_traversable"),
                                        ("leg", "/leg/nav_map", "leg_traversable")):
                m = self._grid_msgs[topic]
                occ_d, occ_xs, occ_ys = occgrid_to_array(m)
                fn = fn_rate(occ_d, occ_xs, occ_ys, self.gt[gtkey], self.gt["xs"], self.gt["ys"])
                self.result[f"{robot}_FN"] = fn
                print('[%s] %s FN%%=%.2f%% (GT통과가능 %d개 중 pipeline이 막힘판정 %d개, '
                     '미측정 %.1f%%)' % (self.tag, robot, fn["FN_percent"],
                                       fn["n_gt_traversable"], fn["n_pipeline_occupied_FN"],
                                       fn["unknown_percent"]))
        with open(self.out_path, "w") as f:
            json.dump(self.result, f, indent=2, ensure_ascii=False)
        print(f"[{self.tag}] 저장: {self.out_path}")
        rclpy.shutdown()


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else '?'
    out_path = sys.argv[2] if len(sys.argv) > 2 else f"/tmp/{tag}_fn.json"
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 90.0
    rclpy.init()
    try:
        rclpy.spin(Capture(tag, out_path, timeout))
    except Exception as e:
        print(f"error: {e}")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
