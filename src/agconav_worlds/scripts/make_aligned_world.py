#!/usr/bin/env python3
"""본 맵을 **도심 격자에 축정렬**시킨 새 월드를 만든다.

    make_aligned_world.py [--out-dir ...] [--margin 0]

왜: `Seongdong_gu` 의 건물 격자는 세계 축에 대해 **−22.03도** 돌아가 있다.
그대로 잔디깎기를 하면 빈 모서리까지 날아 비행 거리의 33% 를 버린다.
경로를 돌리는 대안은 기수 고정(+x) 때문에 라이다 스와스 기하가 바뀌어
축소 월드에서 얻은 간격-품질 보정이 무효가 된다.

그래서 **월드 쪽을 돌린다.** 지형·건물·텍스처를 +22.03도 회전해 도심 장축을
세계 x 에 맞추고, 건물이 없는 바깥을 잘라낸다. 그러면

  - 스트립은 세계 x 축 그대로 (스와스 기하 100% 보존)
  - 탐지 영역이 축정렬 직사각형이라 클리핑 불필요
  - 지형 자체가 도심만 남아 라이다가 빈 곳을 볼 일이 없다

변환: p_new = R(+yaw) @ (p_old - c)     c = 도심 사각형 중심, yaw = +22.03도

Gazebo heightmap 은 **정사각 (2^n)+1 이미지**여야 한다. 1025x1025 를 쓰고
`<size>` 로 실제 범위를 준다. 픽셀 해상도가 원본(0.70 x 0.65 m)보다 조밀해지므로
(0.56 x 0.47 m) 리샘플로 잃는 것은 없다.

Gazebo의 높이 매핑은 16-bit 상수 65535가 아니라 **해당 이미지의 최댓값**을
분모로 쓴다. 따라서 크롭/회전 뒤 원본 픽셀값을 그대로 저장하면 새 이미지의
최댓값이 낮아진 만큼 지형 전체가 부풀어 오른다. 샘플을 실제 월드 고도로 바꾼
뒤 0..65535로 다시 펴고, 그 고도 범위를 `<size>.z`와 `<pos>.z`에 기록한다.
"""
import argparse
import math
import os
import re
import shutil

import numpy as np
from PIL import Image

SRC = 'src/agconav_worlds/worlds/Seongdong_gu'
SIZE_X, SIZE_Y, SIZE_Z = 717.9, 665.36, 18.4
POS_X, POS_Y, POS_Z = 11.39, 1.61, -1.0
BOX = (POS_X - SIZE_X / 2, POS_X + SIZE_X / 2, POS_Y - SIZE_Y / 2, POS_Y + SIZE_Y / 2)
N = 1025


def building_vertices():
    txt = open(os.path.join(SRC, 'mesh/buildings.dae')).read()
    m = re.search(r'<float_array[^>]*id="verts-array-array"[^>]*>(.*?)</float_array>', txt, re.S)
    V = np.fromstring(m.group(1), sep=' ').reshape(-1, 3)
    return V, txt


def min_area_rect(P):
    """볼록껍질 + 회전 캘리퍼스로 최소 넓이 직사각형."""
    p = P[np.lexsort((P[:, 1], P[:, 0]))]

    def half(pts):
        h = []
        for q in pts:
            while len(h) >= 2 and np.cross(h[-1] - h[-2], q - h[-2]) <= 0:
                h.pop()
            h.append(q)
        return h
    H = np.array(half(p)[:-1] + half(p[::-1])[:-1])
    best = None
    for i in range(len(H)):
        e = H[(i + 1) % len(H)] - H[i]
        a = math.atan2(e[1], e[0])
        c, s = math.cos(-a), math.sin(-a)
        Q = H @ np.array([[c, -s], [s, c]]).T
        w, h = Q[:, 0].ptp(), Q[:, 1].ptp()
        if best is None or w * h < best[0]:
            cen = np.array([(Q[:, 0].min() + Q[:, 0].max()) / 2,
                            (Q[:, 1].min() + Q[:, 1].max()) / 2])
            cc, ss = math.cos(a), math.sin(a)
            best = (w * h, a, w, h, np.array([[cc, -ss], [ss, cc]]) @ cen)
    return best[1], best[2], best[3], best[4]


def sample(img, xs, ys):
    """원본 월드 좌표 (xs, ys) 에서 이미지를 이중선형 샘플. 행 0 = 최대 y."""
    a = np.asarray(img).astype(np.float64)
    ny, nx = a.shape[:2]
    c = np.clip((xs - BOX[0]) / (BOX[1] - BOX[0]) * (nx - 1), 0, nx - 1.001)
    r = np.clip((BOX[3] - ys) / (BOX[3] - BOX[2]) * (ny - 1), 0, ny - 1.001)
    c0, r0 = np.floor(c).astype(int), np.floor(r).astype(int)
    fc, fr = (c - c0)[..., None], (r - r0)[..., None]
    if a.ndim == 2:
        a = a[..., None]
        fc, fr = fc[..., 0][..., None], fr[..., 0][..., None]
    v = (a[r0, c0] * (1 - fc) * (1 - fr) + a[r0, c0 + 1] * fc * (1 - fr)
         + a[r0 + 1, c0] * (1 - fc) * fr + a[r0 + 1, c0 + 1] * fc * fr)
    return v[..., 0] if v.shape[-1] == 1 else v


def transform_world(src_world, out_path, yaw, cen, LX, LY, SX, SY, SIZE_Z, POS_Z,
                    drone_z):
    """월드 파일의 좌표를 전부 새 프레임으로 옮긴다. **삭제는 하지 않는다.**"""
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])          # p_new = (p_old - cen) @ R

    def tf_pose(txt):
        v = [float(x) for x in txt.split()]
        q = (np.array(v[:2]) - cen) @ R
        v[0], v[1] = q[0], q[1]
        if len(v) >= 6:
            v[5] -= yaw
        return ' '.join('%.6f' % x for x in v)

    t = src_world
    t = re.sub(r'<world name="[^"]*">', '<world name="Seongdong_gu">', t, count=1)
    t = t.replace('<size>717.9 665.36 18.4</size>',
                  '<size>%.3f %.3f %.6f</size>' % (LX, LY, SIZE_Z))
    t = t.replace('<pos>11.39 1.61 -1.0</pos>',
                  '<pos>0 0 %.6f</pos>' % POS_Z)
    t = t.replace('<pose>11.39 1.61 -1.0 0 0 0</pose>',
                  '<pose>0 0 %.6f 0 0 0</pose>' % POS_Z)
    t = re.sub(r'(<texture>.*?<size>)717\.9(</size>)',
               lambda m: m.group(1) + ('%.3f' % LX) + m.group(2), t, flags=re.S)
    t = t.replace('<pose>11.39 1.61 0 0 0 0</pose>', '<pose>0 0 0 0 0 0</pose>')
    # **<include> 블록 전체를 잡아 그 안의 첫 <pose> 를 변환한다.**
    # 요소 순서에 의존하면 안 된다 -- 원본에 3가지 순서가 섞여 있다:
    #   (name, uri, pose, scale)  나무 72개
    #   (uri, pose)               방지턱/경사로 6개   <- 처음에 이걸 놓쳐서
    #   (uri, name, pose, ...)    드론 X3 1개            원본 좌표에 남아 있었다
    n = [0]
    dropped = []

    def sub_block(m):
        blk = m.group(0)
        pm = re.search(r'<pose>([^<]*)</pose>', blk)
        if not pm:
            return blk
        newp = tf_pose(pm.group(1))
        v = [float(x) for x in newp.split()]
        if '<name>X3</name>' in blk or 'model://agconav_drone' in blk:
            v[2] = drone_z
            newp = ' '.join('%.6f' % x for x in v)
        if abs(v[0]) > LX / 2 or abs(v[1]) > LY / 2:
            nm = re.search(r'<name>([^<]*)</name>', blk)
            uri = re.search(r'<uri>\s*([^<\s]+)', blk)
            dropped.append((nm.group(1) if nm else (uri.group(1) if uri else '?'),
                            v[0], v[1]))
            return ''
        n[0] += 1
        return blk[:pm.start()] + '<pose>' + newp + '</pose>' + blk[pm.end():]

    t = re.sub(r'<include>.*?</include>', sub_block, t, flags=re.S)
    print('  <include> pose 변환 %d개' % n[0])
    if dropped:
        print('  크롭 밖이라 제외 %d개 (지형이 없어 공중에 뜬다):' % len(dropped))
        for nm, x, y in dropped:
            print('    %-14s (%.1f, %.1f)  경계 밖 %.0f m'
                  % (nm, x, y, max(abs(x) - LX / 2, abs(y) - LY / 2)))
    t = re.sub(r'<camera_pose>[^<]*</camera_pose>',
               '<camera_pose>0 -%.0f %.0f 0 0.75 1.5708</camera_pose>' % (LY * 0.8, LY * 0.8), t)
    open(out_path, 'w').write(t)
    return n[0]


def main(out_dir, margin):
    V, dae = building_vertices()
    P = V[:, :2] + [POS_X, POS_Y]                 # 건물 mesh pose 11.39 1.61
    yaw, w, h, cen = min_area_rect(P)
    SX, SY = w + 2 * margin, h + 2 * margin       # **탐지 박스** = 건물 영역
    print('도심 사각형(탐지 박스): %.1f x %.1f m, yaw %.3f도, 중심 (%.2f, %.2f)'
          % (SX, SY, math.degrees(yaw), cen[0], cen[1]))

    # 월드 = 도심 사각형 그대로. 건물이 없는 바깥은 잘라낸다.
    # 잘리는 객체는 삭제하지 않고 **목록으로 찍는다** -- 조용히 지우지 않기 위해서.
    LX, LY = SX, SY
    print('새 월드    : %.1f x %.1f m (도심만 남기고 빈 바깥은 잘라냄)' % (LX, LY))

    # 새 좌표 -> 원본 좌표:  p_old = R(yaw) @ p_new + cen
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])

    mesh = os.path.join(out_dir, 'mesh')
    os.makedirs(mesh, exist_ok=True)

    nx = np.linspace(-LX / 2, LX / 2, N)
    ny = np.linspace(LY / 2, -LY / 2, N)           # 행 0 = 최대 y
    GX, GY = np.meshgrid(nx, ny)
    old = np.stack([GX, GY], -1) @ R.T + cen

    hm = Image.open(os.path.join(SRC, 'mesh/height_map.png'))
    src_pixels = np.asarray(hm).astype(np.float64)
    out = sample(hm, old[..., 0], old[..., 1])
    # gz-common ImageHeightmap은 이미지 안의 최댓값으로 정규화한다. 회전 크롭의
    # max(현재 63507)를 원본 max(65474)처럼 취급하게 두면 지면이 최대 약 0.57 m
    # 솟아 wheel/Go2를 덮는다. 실제 고도로 환산한 뒤 전체 16-bit 범위로 편다.
    elev = out / src_pixels.max() * SIZE_Z + POS_Z
    new_pos_z = float(elev.min())
    new_size_z = float(elev.max() - elev.min())
    scaled = (elev - new_pos_z) / new_size_z * 65535.0
    Image.fromarray(np.round(scaled).astype(np.uint16)).save(
        os.path.join(mesh, 'height_map.png'))
    print('height_map.png  %dx%d  GT %.3f ~ %.3f m -> size.z %.6f pos.z %.6f'
          % (N, N, elev.min(), elev.max(), new_size_z, new_pos_z))

    # 텍스처는 **지형 범위 그대로** 샘플한다. 이미지 크기도 원본과 같게 둔다.
    # 원본이 정상 동작하므로 규약을 하나도 바꾸지 않는 것이 원칙이다.
    # (한때 정사각으로 샘플했다가 되돌렸다 -- SDF <texture><size> 의 UV 기준점을
    #  확인하지 않고 가정한, 근거 없는 변경이었다.)
    for f in ('aerial.png', 'normal_map.png'):
        src = os.path.join(SRC, 'mesh', f)
        if not os.path.exists(src):
            continue
        im = Image.open(src).convert('RGB')
        M = im.size[0]                      # 원본 이미지 크기 그대로
        mx = np.linspace(-LX / 2, LX / 2, M)
        my = np.linspace(LY / 2, -LY / 2, M)
        MGX, MGY = np.meshgrid(mx, my)
        mold = np.stack([MGX, MGY], -1) @ R.T + cen
        o = sample(im, mold[..., 0], mold[..., 1])
        Image.fromarray(np.clip(o, 0, 255).astype(np.uint8)).save(os.path.join(mesh, f))
        print('%-16s %dx%d  (지형 범위 %.1f x %.1f m, 원본과 같은 규약)'
              % (f, M, M, LX, LY))

    # 건물: p_new = R^T @ (p_old - cen).  법선은 회전만.
    Vn = V.copy()
    Vn[:, :2] = (P - cen) @ R
    body = ' '.join('%.4f' % v for v in Vn.reshape(-1))
    dae2 = re.sub(r'(<float_array[^>]*id="verts-array-array"[^>]*>).*?(</float_array>)',
                  lambda m: m.group(1) + body + m.group(2), dae, count=1, flags=re.S)
    mm = re.search(r'<float_array[^>]*id="normals-array-array"[^>]*>(.*?)</float_array>', dae, re.S)
    if mm:
        Nv = np.fromstring(mm.group(1), sep=' ').reshape(-1, 3)
        Nv[:, :2] = Nv[:, :2] @ R
        nb = ' '.join('%.4f' % v for v in Nv.reshape(-1))
        dae2 = re.sub(r'(<float_array[^>]*id="normals-array-array"[^>]*>).*?(</float_array>)',
                      lambda m: m.group(1) + nb + m.group(2), dae2, count=1, flags=re.S)
    open(os.path.join(mesh, 'buildings.dae'), 'w').write(dae2)
    Q = Vn[:, :2]
    print('buildings.dae   정점 %d개  x %.1f~%.1f  y %.1f~%.1f'
          % (len(Vn), Q[:, 0].min(), Q[:, 0].max(), Q[:, 1].min(), Q[:, 1].max()))

    # 스폰 pose 변환. z는 예전 상수 6.15를 복사하지 않고 원본 heightmap GT에서
    # 직접 구한다. 축정렬 지형은 이 GT를 보존하므로 wheel/Go2가 지면에 묻히지
    # 않으면서 원본과 동일한 차체 하부 여유를 갖는다.
    def tf(x, y, yaw_old):
        p = (np.array([x, y]) - cen) @ R
        return p[0], p[1], yaw_old - yaw

    def source_ground(x, y):
        pixel = float(sample(hm, np.array(x), np.array(y)))
        return pixel / src_pixels.max() * SIZE_Z + POS_Z

    # x, y, 지면 위 여유, yaw. 드론은 gravity=1인 실제 비행 모델이므로
    # 컨트롤러/브리지 시작 전 낙하 여유를 0.65 m 둔다.
    SPAWNS = {'drone': (-156.4800, 148.1800, 0.6500, -0.3897),
              'wheel': (-159.6820, 147.5380, 0.2500, -0.4349),   # A300
              'leg':   (-158.8710, 150.8310, 0.3000, -0.4613)}   # Go2
    newsp = {}
    for k, (x, y, clearance, yw) in SPAWNS.items():
        nxp, nyp, nyw = tf(x, y, yw)
        z = source_ground(x, y) + clearance
        newsp[k] = (nxp, nyp, z, nyw)
        print('  %-6s GT %.4f + 여유 %.3f -> 스폰 (%9.4f, %9.4f, z %.4f, yaw %7.4f)'
              % (k, z - clearance, clearance, nxp, nyp, z, nyw))
    dx, dy, dyaw = newsp["drone"][0], newsp["drone"][1], newsp["drone"][3]

    src_world = open(os.path.join(SRC, 'Seongdong_gu.world')).read()
    transform_world(src_world, os.path.join(out_dir, 'Seongdong_gu_aligned.world'),
                    yaw, cen, LX, LY, SX, SY, new_size_z, new_pos_z,
                    newsp['drone'][2])

    with open(os.path.join(out_dir, 'aligned_params.txt'), 'w') as f:
        f.write('yaw_deg %.6f\ncenter %.6f %.6f\nsize %.3f %.3f %.3f\n'
                'survey %.3f %.3f\npos_z %.3f\n'
                % (math.degrees(yaw), cen[0], cen[1], LX, LY, new_size_z,
                   SX, SY, new_pos_z))
        for k, (a, b, cz, d) in newsp.items():
            f.write('%s_spawn %.4f %.4f %.4f %.4f\n' % (k, a, b, cz, d))
    return LX, LY, dx, dy, dyaw


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir',
                   default='src/agconav_worlds/worlds/Seongdong_gu_aligned')
    p.add_argument('--margin', type=float, default=0.0)
    a = p.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    main(a.out_dir, a.margin)
