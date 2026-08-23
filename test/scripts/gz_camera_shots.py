#!/usr/bin/env python3
"""gz GUI 카메라를 정해진 시점들로 옮기며 스크린샷을 받는다.

    gz_camera_shots.py <출력 디렉터리>

`capture_gz_shots.sh` 가 GUI 를 띄운 뒤 호출한다. 단독으로도 쓸 수 있다
(GUI 가 이미 떠 있어야 한다).

/gui/screenshot 은 **디렉터리**를 받아 자기가 타임스탬프 파일명을 만든다.
전체 경로를 주면 true 만 돌려주고 아무것도 저장하지 않는다.
"""
import math
import os
import subprocess
import sys
import time

# 3로봇 스폰 좌표 (agconav_all.launch.py 기본값 / 월드의 X3 pose).
#   wheel (-195.21, 73.02)  leg (-195.70, 76.37)  drone X3 (-192.49, 74.81)
TARGET = (-194.3, 75.0, 6.6)
VIEWS = [
    (-198.2, 71.0, 8.1),   # 남서 낮게 — 스카이라인이 배경에 들어온다
    (-196.8, 69.8, 9.2),   # 남서 높게
    (-190.6, 71.0, 8.1),   # 남동
    (-194.3, 69.4, 8.6),   # 정남
    (-199.5, 73.5, 8.4),   # 서
]


def look_at(cam, tgt):
    """cam 에서 tgt 를 바라보는 자세를 (w, x, y, z) 로. roll 은 0."""
    dx, dy, dz = (t - c for c, t in zip(cam, tgt))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(dz, math.hypot(dx, dy))
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(-pitch / 2), math.sin(-pitch / 2)
    return cy * cp, -sy * sp, cy * sp, sy * cp


def gz(service, reqtype, req):
    return subprocess.run(
        ['gz', 'service', '-s', service, '--reqtype', reqtype,
         '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', req],
        check=False, capture_output=True, text=True)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '.'
    os.makedirs(out, exist_ok=True)
    for i, cam in enumerate(VIEWS):
        w, x, y, z = look_at(cam, TARGET)
        gz('/gui/move_to/pose', 'gz.msgs.GUICamera',
           'pose: {position: {x: %.4f, y: %.4f, z: %.4f}, '
           'orientation: {w: %.6f, x: %.6f, y: %.6f, z: %.6f}}'
           % (cam[0], cam[1], cam[2], w, x, y, z))
        time.sleep(5)                      # 카메라 이동 + 렌더 안정화
        gz('/gui/screenshot', 'gz.msgs.StringMsg', 'data: "%s"' % out)
        time.sleep(3)
        print('view %d %s' % (i, cam))


if __name__ == '__main__':
    main()
