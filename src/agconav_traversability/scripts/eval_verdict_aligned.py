#!/usr/bin/env python3
"""**정렬 월드**(Seongdong_gu_aligned)에서 주행성 판정 정확도를 정답 지형과 대조한다.

계산은 `eval_verdict_accuracy.py` 와 같고 월드 상수만 다르다.
상수는 파일에 박지 않고 `aligned_params.txt` 에서 읽는다 -- 월드를 다시 만들면
size/pos_z 가 바뀌기 때문이다(실제로 z 매핑이 18.4/-1.0 -> 17.823/-0.976 으로
재보정된 적이 있다).
"""
import argparse
import json
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_verdict_accuracy import (LEG_STEP, WHEEL_STEP, load_map,  # noqa: E402
                                   max_step)

WDIR = 'src/agconav_worlds/worlds/Seongdong_gu_aligned'
GROUND_TOL = 0.8


def params(wdir):
    p = dict(l.split(None, 1) for l in open(os.path.join(wdir, 'aligned_params.txt')))
    lx, ly, sz = (float(v) for v in p['size'].split())
    return lx, ly, sz, float(p['pos_z'])


def reference_grid(path, xs, ys, lx, ly, sz, pz):
    img = np.array(Image.open(path)).astype(np.float64)
    ny, nx = img.shape
    box = (-lx / 2, lx / 2, -ly / 2, ly / 2)
    X, Y = np.meshgrid(xs, ys)
    c = np.clip((X - box[0]) / (box[1] - box[0]) * (nx - 1), 0, nx - 2)
    r = np.clip((box[3] - Y) / (box[3] - box[2]) * (ny - 1), 0, ny - 2)
    c0, r0 = np.floor(c).astype(int), np.floor(r).astype(int)
    fc, fr = c - c0, r - r0
    v = (img[r0, c0] * (1 - fc) * (1 - fr) + img[r0, c0 + 1] * fc * (1 - fr)
         + img[r0 + 1, c0] * (1 - fc) * fr + img[r0 + 1, c0 + 1] * fc * fr)
    return v / 65535.0 * sz + pz


def main(uri, tag, wdir, box=None):
    lx, ly, sz, pz = params(wdir)
    z, xs, ys, _ = load_map(uri)
    ref = reference_grid(os.path.join(wdir, 'mesh/height_map.png'), xs, ys, lx, ly, sz, pz)

    X, Y = np.meshgrid(xs, ys)
    if box:
        inbox = (X >= box[0]) & (X <= box[1]) & (Y >= box[2]) & (Y <= box[3])
    else:
        inbox = (np.abs(X) <= lx / 2) & (np.abs(Y) <= ly / 2)
    meas_ok = np.isfinite(z) & inbox
    ground = meas_ok & (np.abs(z - ref) <= GROUND_TOL)

    m_step, m_have = max_step(np.where(meas_ok, z, 0.0), meas_ok)
    t_step, t_have = max_step(ref, np.ones(ref.shape, bool))
    ok = ground & m_have & t_have
    n = int(ok.sum())
    if n == 0:
        print(json.dumps(dict(tag=tag, cells=0)))
        return
    ms, ts = m_step[ok], t_step[ok]
    out = dict(tag=tag, cells=n, inbox_cells=int(inbox.sum()),
               observed_cells=int(meas_ok.sum()), ground_cells=int(ground.sum()),
               coverage=float(100.0 * meas_ok.sum() / inbox.sum()),
               ground_frac=float(100.0 * ground.sum() / max(1, meas_ok.sum())),
               height_err_rms=float(np.sqrt(((z[ok] - ref[ok]) ** 2).mean())),
               true_step_median=float(np.median(ts)),
               meas_step_median=float(np.median(ms)))
    for name, th in (('wheel', WHEEL_STEP), ('leg', LEG_STEP)):
        truth = ts <= th
        pred = ms <= th
        tp = int((truth & pred).sum()); fp = int((~truth & pred).sum())
        fn = int((truth & ~pred).sum()); tn = int((~truth & ~pred).sum())
        out[name] = dict(true_pass_pct=float(100.0 * truth.mean()),
                         pred_pass_pct=float(100.0 * pred.mean()),
                         accuracy=float(100.0 * (tp + tn) / n), fp=fp, fn=fn,
                         fn_rate=float(100.0 * fn / max(1, truth.sum())),
                         precision=float(100.0 * tp / max(1, tp + fp)))
    print(json.dumps(out))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('map'); p.add_argument('--tag', default='?')
    p.add_argument('--world-dir', default=WDIR, dest='wdir')
    p.add_argument('--box', nargs=4, type=float, default=None,
                   help='평가 박스 x0 x1 y0 y1 (생략하면 월드 전체)')
    a = p.parse_args()
    main(a.map, a.tag, a.wdir, tuple(a.box) if a.box else None)
