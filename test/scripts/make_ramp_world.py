#!/usr/bin/env python3
"""실험 1용 **단계식 경사로** 월드를 만든다.

    make_ramp_world.py <출력.sdf>

5, 10, 15, 20, 25, 30도 구간을 각각 수평 4 m 씩 이어 붙인 하나의 경사로다.
로봇을 x=0 에서 +x 로 밀면 오르다가 못 오르는 각도에서 멈춘다. 멈춘 x 가
곧 그 컨트롤러의 **등판 한계 각도**다.

왜 각도별로 월드를 따로 안 쓰나: 각도 하나에 한 번씩 재면 조합이 6배로
늘어난다. 한 번 실행에 7분이 걸려서 4 컨트롤러 x 6 각도 x 3회면 8시간이다.
단계식은 한 번에 한계가 나오므로 12회면 끝난다. 재는 값도 이쪽이 질문에
더 가깝다 — 우리가 알고 싶은 것은 "몇 도까지 오르는가"지 "15도에서
통과했는가"가 아니다.

폭은 20 m 로 넉넉히 준다. 6 m 로 했더니 요 드리프트로 옆으로 벗어났다.
"""
import math
import sys

SEGMENTS = [5, 10, 15, 20, 25, 30]   # 각도 [도]
SEG_RUN = 4.0                        # 구간별 수평 투영 길이 [m]
START = 3.0                          # 경사 시작 x
WIDTH = 20.0
THICK = 0.4
TOP = 4.0                            # 정상부 평지

HEAD = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="Seongdong_gu">
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
    <model name="ground">
      <static>true</static>
      <pose>0 0 -0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>300 60 1</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>300 60 1</size></box></geometry>
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


def segment_table():
    """[(x_start, x_end, z_start, 각도), ...] 와 정상부 높이를 돌려준다.

    시험 스크립트가 지형 높이를 계산할 때 이 표와 **같은 값**을 써야 한다.
    """
    rows, x, z = [], START, 0.0
    for deg in SEGMENTS:
        dz = SEG_RUN * math.tan(math.radians(deg))
        rows.append((x, x + SEG_RUN, z, deg))
        x += SEG_RUN
        z += dz
    return rows, z


def main(out):
    rows, top_z = segment_table()
    body = ''
    for x0, x1, z0, deg in rows:
        t = math.radians(deg)
        length = SEG_RUN / math.cos(t)
        # 윗면 중점이 (중앙, 0, z0 + dz/2) 가 되도록 박스 중심을 법선 반대로 내린다.
        dz = SEG_RUN * math.tan(t)
        nx, nz = -math.sin(t), math.cos(t)
        cx = (x0 + x1) / 2 + (THICK / 2) * nx
        cz = z0 + dz / 2 - (THICK / 2) * nz
        body += box('ramp%d' % deg, (length, WIDTH, THICK),
                    f'{cx} 0 {cz} 0 {-t} 0')
    body += box('summit', (TOP, WIDTH, THICK),
                f'{START + SEG_RUN * len(SEGMENTS) + TOP / 2} 0 {top_z - THICK / 2} 0 0 0')
    open(out, 'w').write(HEAD + body + TAIL)
    print('단계식 경사로 -> %s' % out)
    for x0, x1, z0, deg in rows:
        print('  %2d도  x %5.1f ~ %5.1f  시작높이 %5.2f m' % (deg, x0, x1, z0))
    print('  정상부 높이 %.2f m (x >= %.1f)'
          % (top_z, START + SEG_RUN * len(SEGMENTS)))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'test/worlds/ramp.sdf')
