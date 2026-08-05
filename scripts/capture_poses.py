#!/usr/bin/env python3
"""실행 중인 시뮬레이션에서 로봇·카메라의 현재 위치를 읽어 설정값으로 출력한다.

Gazebo GUI에서 마우스로 로봇을 옮기거나(상단 툴바의 이동 도구) 시점을 돌린 뒤
이 스크립트를 실행하면, 그 위치를 그대로 붙여넣을 수 있는 형태로 뽑아준다.

사용법:
    ./scripts/capture_poses.py            # 현재 위치를 설정값으로 출력
    ./scripts/capture_poses.py --apply    # 파일에 직접 반영 (백업 후 수정)

내보내는 것:
  - wheel/leg  -> agconav_sim.launch.py의 launch 인자 형태
  - X3(드론)   -> Seongdong_gu.world의 <pose> 줄
  - GUI 카메라 -> Seongdong_gu.world의 <camera_pose> 줄
"""
import argparse
import math
import pathlib
import re
import shutil
import subprocess
import sys

WORLD = pathlib.Path(__file__).resolve().parent.parent / (
    'src/agconav_worlds/worlds/Seongdong_gu/Seongdong_gu.world')
LAUNCH = pathlib.Path(__file__).resolve().parent.parent / (
    'src/agconav_bringup/launch/agconav_sim.launch.py')

ROBOTS = {'wheel': 'wheel/robot', 'leg': 'leg', 'drone': 'X3'}


def run(cmd, timeout=20):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ''


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def parse_pose_block(text):
    """gz 출력에서 position/orientation을 뽑는다."""
    nums = {}
    section = None
    for line in text.splitlines():
        line = line.strip()
        if line.startswith('position'):
            section = 'p'
        elif line.startswith('orientation'):
            section = 'o'
        else:
            m = re.match(r'([xyzw]):\s*(-?[\d.eE+-]+)', line)
            if m and section:
                nums[section + m.group(1)] = float(m.group(2))
    if 'px' not in nums:
        return None
    return (
        (nums.get('px', 0.0), nums.get('py', 0.0), nums.get('pz', 0.0)),
        (nums.get('ox', 0.0), nums.get('oy', 0.0), nums.get('oz', 0.0),
         nums.get('ow', 1.0)),
    )


def model_pose(name):
    out = run(f'gz model -m "{name}" -p')
    m = re.search(r'\[\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*\]\s*'
                  r'\[\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*\]', out)
    if not m:
        return None
    v = [float(g) for g in m.groups()]
    return (v[0], v[1], v[2]), (v[3], v[4], v[5])


def camera_pose():
    out = run('gz topic -e -t /gui/camera/pose -n 1', timeout=25)
    parsed = parse_pose_block(out)
    if not parsed:
        return None
    pos, quat = parsed
    return pos, quat_to_rpy(*quat)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true',
                    help='출력만 하지 않고 world/launch 파일에 직접 반영한다')
    args = ap.parse_args()

    poses = {k: model_pose(n) for k, n in ROBOTS.items()}
    cam = camera_pose()

    missing = [k for k, v in poses.items() if v is None]
    if missing or cam is None:
        print('시뮬레이션에서 위치를 읽지 못했습니다: '
              + ', '.join(missing + ([] if cam else ['카메라'])), file=sys.stderr)
        print('시뮬레이션이 실행 중인지, ROS_DOMAIN_ID=42 환경인지 확인하세요.',
              file=sys.stderr)
        return 1

    print('=' * 72)
    print('현재 위치')
    print('=' * 72)
    for k, (p, r) in poses.items():
        print(f'  {k:6s} ({p[0]:9.3f}, {p[1]:9.3f}, {p[2]:7.3f})  yaw {r[2]:+.4f}')
    print(f'  camera ({cam[0][0]:9.3f}, {cam[0][1]:9.3f}, {cam[0][2]:7.3f})  '
          f'pitch {cam[1][1]:+.4f} yaw {cam[1][2]:+.4f}')

    wp, wr = poses['wheel']
    lp, lr = poses['leg']
    dp, dr = poses['drone']
    cam_line = (f'<camera_pose>{cam[0][0]:.2f} {cam[0][1]:.2f} {cam[0][2]:.2f} '
                f'0 {cam[1][1]:.4f} {cam[1][2]:.4f}</camera_pose>')
    drone_line = (f'<pose>{dp[0]:.2f} {dp[1]:.2f} {dp[2]:.2f} '
                  f'{dr[0]:.4f} {dr[1]:.4f} {dr[2]:.4f}</pose>')

    print('\n' + '=' * 72)
    print('1) 이번 실행만 이 위치로 띄우기 — 그대로 복사해서 실행')
    print('=' * 72)
    print(f'ros2 launch agconav_bringup agconav_sim.launch.py \\\n'
          f'  wheel_x:={wp[0]:.2f} wheel_y:={wp[1]:.2f} wheel_z:={wp[2] + 0.4:.2f} '
          f'wheel_yaw:={wr[2]:.4f} \\\n'
          f'  leg_x:={lp[0]:.2f} leg_y:={lp[1]:.2f} leg_z:={lp[2] + 0.4:.2f} '
          f'leg_yaw:={lr[2]:.4f}')

    print('\n' + '=' * 72)
    print('2) 기본값으로 굳히기 — 아래를 파일에 반영 (--apply 로 자동 반영 가능)')
    print('=' * 72)
    print(f'  {LAUNCH.name}  기본값: wheel ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2] + 0.4:.2f}), '
          f'leg ({lp[0]:.2f}, {lp[1]:.2f}, {lp[2] + 0.4:.2f})')
    print(f'  {WORLD.name}  X3 <pose>:   {drone_line}')
    print(f'  {WORLD.name}  카메라:      {cam_line}')

    if not args.apply:
        print('\n(--apply 를 붙이면 위 값을 파일에 직접 씁니다)')
        return 0

    # --- 파일 반영 ---------------------------------------------------------
    for path in (WORLD, LAUNCH):
        shutil.copy(path, str(path) + '.bak')
    print(f'\n백업 생성: {WORLD.name}.bak, {LAUNCH.name}.bak')

    w = WORLD.read_text()
    w, n1 = re.subn(r'<camera_pose>[^<]*</camera_pose>', cam_line, w, count=1)
    w, n2 = re.subn(r'(<name>X3</name>\s*\n\s*)<pose>[^<]*</pose>',
                    lambda m: m.group(1) + drone_line, w, count=1)
    WORLD.write_text(w)

    t = LAUNCH.read_text()
    n3 = 0
    for key, val in (('wheel_x', wp[0]), ('wheel_y', wp[1]), ('wheel_z', wp[2] + 0.4),
                     ('wheel_yaw', wr[2]), ('leg_x', lp[0]), ('leg_y', lp[1]),
                     ('leg_z', lp[2] + 0.4), ('leg_yaw', lr[2])):
        t, c = re.subn(rf'\("{key}", "[^"]*"', f'("{key}", "{val:.4f}"', t, count=1)
        n3 += c
    LAUNCH.write_text(t)

    print(f'반영 완료 — 카메라 {n1}건, 드론 {n2}건, launch 기본값 {n3}건')
    print('colcon build --symlink-install --packages-select '
          'agconav_worlds agconav_bringup 후 다시 실행하세요.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
