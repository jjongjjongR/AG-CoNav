#!/usr/bin/env python3
"""실험 1용 경사/단차 월드를 만든다.

!! 폭은 넉넉해야 한다 !!
처음엔 6 m 로 만들었는데, RL 정책이 직진 명령만 받으면 요가 계속 틀어져
8 m 가는 동안 옆으로 4~4.8 m 밀린다. 그러면 램프 옆으로 벗어나 평지에
내려서는데, 판정은 그 x 에서 램프 높이를 기대하므로 정상 보행을 "전복"으로
오판했다(실측 slope10 robot_lab 3/3 이 그렇게 찍혔다). 20 m 로 넓혔고,
시험 스크립트도 y=0 으로 되돌리는 조향을 함께 넣었다.

    make_terrain_worlds.py <출력 디렉터리>

경사: x=3.0 에서 시작해 수평 투영 6 m 를 올라가고, 그 뒤 6 m 평지(정상부).
단차: x=3.0 에서 시작하는 폭 6 m, 높이 h 의 박스.

둘 다 x=0 원점에 로봇을 놓고 +x 로 밀어 넘게 한다. 정상부를 둔 이유는
"올라탔다"와 "넘었다"를 구분하기 위해서다 — 경사 끝에서 미끄러져 내려오면
정상부 도달 거리에 못 미친다.

문서 12 의 wheel 시험과 같은 형상이라 leg/wheel 결과를 그대로 비교할 수 있다.
"""
import math
import os
import sys

HEAD = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="Seongdong_gu">
    <!-- 운용 월드(Seongdong_gu_aligned)와 같은 물리 설정을 쓴다.
         기본 충돌 검출기로는 Go2 가 RL 진입 직후 옆으로 넘어졌다
         (IMU 기울기 94도). 발 접촉이 많은 4족은 검출기 영향을 크게 받는다. -->
    <physics name="1ms" type="ignored">
      <dart>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-forcetorque-system"
            name="gz::sim::systems::ForceTorque"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <!-- 지면은 무한 plane 이 아니라 큰 박스를 쓴다. plane 은 4족 발 접촉에서
         불안정했다(RL 진입 직후 전복). 박스는 접촉 판정이 단순하고 안정적이다. -->
    <model name="ground">
      <static>true</static>
      <pose>0 0 -0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>200 60 1</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>200 60 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

TAIL = """  </world>
</sdf>
"""


def box(name, size, pose, mu=1.0):
    sx, sy, sz = size
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>0.3 0.3 0.4 1</ambient><diffuse>0.4 0.4 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def slope_world(deg, run=6.0, start=3.0, width=20.0, thick=0.4, top=6.0):
    """수평 투영 run m 를 deg 로 올라가는 램프 + 정상부."""
    if deg == 0:
        body = box('ramp', (run + top, width, thick),
                   f'{start + (run + top) / 2} 0 {-thick / 2} 0 0 0')
        return HEAD + body + TAIL
    t = math.radians(deg)
    length = run / math.cos(t)                     # 경사면 실제 길이
    height = run * math.tan(t)                     # 정상부 높이
    # 윗면 중점이 (start+run/2, 0, height/2) 가 되도록 박스 중심을 내린다.
    nx, nz = -math.sin(t), math.cos(t)             # 윗면 법선
    cx = start + run / 2 + (thick / 2) * nx
    cz = height / 2 + (thick / 2) * (-nz)
    body = box('ramp', (length, width, thick), f'{cx} 0 {cz} 0 {-t} 0')
    # 정상부: 램프 끝에서 이어지는 평지
    body += box('summit', (top, width, thick),
                f'{start + run + top / 2} 0 {height - thick / 2} 0 0 0')
    return HEAD + body + TAIL


def step_world(mm, start=3.0, run=6.0, width=20.0):
    h = mm / 1000.0
    body = box('step', (run, width, h), f'{start + run / 2} 0 {h / 2} 0 0 0')
    return HEAD + body + TAIL


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'test/worlds'
    os.makedirs(out, exist_ok=True)
    made = []
    for d in (0, 10, 15, 20, 25, 30):
        p = os.path.join(out, f'slope{d}.sdf')
        open(p, 'w').write(slope_world(d))
        made.append((f'slope{d}', f'{d}°, 정상부 높이 {6*math.tan(math.radians(d)):.2f} m'))
    for mm in (50, 100, 150, 200, 250, 300):
        p = os.path.join(out, f'step{mm}.sdf')
        open(p, 'w').write(step_world(mm))
        made.append((f'step{mm}', f'{mm} mm'))
    for n, d in made:
        print(f'  {n:10s} {d}')
    print(f'{len(made)}개 생성 -> {out}')


if __name__ == '__main__':
    main()
