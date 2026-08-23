"""실험 1: Go2 를 이미 떠 있는 월드에 띄우고 지정한 컨트롤러로 걷게 한다.

    ros2 launch test/scripts/leg_bench.launch.py controller:=rl policy:=robot_lab

이 파일은 운용 스폰(agconav_bringup/launch/go2_rl_spawn.launch.py)을 그대로
본떴다. **의도적으로 베낀 것이다** — 2차 세션에서 직접 짠 런치로는 RL 진입
직후 로봇이 옆으로 넘어졌는데(28회 전부), 같은 컨트롤러·정책이 운용
스택에서는 잘 걸었다. 컨트롤러 성능을 재는 실험이므로 스폰 쪽 조건은
운용과 한 글자도 다르면 안 된다. 운용 런치를 고치면 여기도 같이 봐야 한다.

운용과 다른 점은 계측에 필요한 세 가지뿐이다.
  1. controller 인자로 unitree_guide_controller 도 띄운다(실험 1의 비교 대상).
  2. 중계는 bench_relay.py 를 쓴다(guide 는 보행 진입 명령과 축 단위가 다르다).
  3. 월드 dynamic_pose 를 브리지해 **정답 3D 포즈**를 받는다.
     운용 xacro 의 OdometryPublisher 에는 <dimensions> 가 없어 2D 라 z 가 항상
     0 이다. 이 실험은 몸통 높이와 등판 높이가 판정 기준이라 z 가 있어야 한다.
     모델을 고치지 않고 월드 쪽에서 받는 편이 운용 조건을 안 건드린다.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

TEST_DIR = os.path.dirname(os.path.abspath(__file__))

# gazebo.yaml 의 **rl_quadruped_controller** down_pos 를 관절 이름으로 푼 것.
# !! joints: 목록이 컨트롤러마다 따로 있고 순서가 다르다 !!
#   unitree_guide_controller / rl_quadruped_controller 둘 다 FR, FL, RR, RL 이지만
#   과거에 FL, FR, RL, RR 로 읽어 hip 부호 4개가 전부 뒤집힌 적이 있다
#   (커밋 d88d20d). 뒷다리가 반대로 벌어진 채 생성돼 보행 진입 때 몸이 틀어진다.
_INITIAL_DOWN_POSITIONS = {
    'FR_hip_joint': 0.01, 'FR_thigh_joint': 1.27, 'FR_calf_joint': -2.8,
    'FL_hip_joint': -0.01, 'FL_thigh_joint': 1.27, 'FL_calf_joint': -2.8,
    'RR_hip_joint': 0.3, 'RR_thigh_joint': 1.31, 'RR_calf_joint': -2.8,
    'RL_hip_joint': -0.3, 'RL_thigh_joint': 1.31, 'RL_calf_joint': -2.8,
}

# gazebo.yaml 의 rl_quadruped_controller **stand_pos**. 우리가 실제로 생성에
# 쓰는 자세는 이쪽이다.
#
# !! 웅크린 자세(down_pos)로 생성하면 안 된다 !!
# down_pos 는 calf -2.8 로 정강이를 완전히 접어 배 밑에 깔고 앉는 자세다.
# 그 자세로 지면에 놓이면 몸통이 정강이 위에 얹히고, 그 상태에서 FIXEDSTAND
# 로 무릎을 펴려 하면 정강이가 몸무게에 눌려 못 펴진다 — 실측: 관절은
# 정확히 기립 자세(hip 0.03, thigh 0.80, calf -1.57)까지 갔는데 몸통은
# 0.114 m 에 그대로 있었다(정상 기립 0.33 m). 발이 지면 아래로 내려간 셈이다.
# 기립 자세로 생성하면 처음부터 발로 서 있으므로 그 함정이 없다.
_INITIAL_STAND_POSITIONS = {
    'FR_hip_joint': 0.0, 'FR_thigh_joint': 0.8, 'FR_calf_joint': -1.5,
    'FL_hip_joint': 0.0, 'FL_thigh_joint': 0.8, 'FL_calf_joint': -1.5,
    'RR_hip_joint': 0.0, 'RR_thigh_joint': 0.8, 'RR_calf_joint': -1.5,
    'RL_hip_joint': 0.0, 'RL_thigh_joint': 0.8, 'RL_calf_joint': -1.5,
}
_SPAWN_POSE = _INITIAL_STAND_POSITIONS

def _set_initial_value(document, interface, value):
    """<param name="initial_value">v</param> 를 state_interface 안에 넣는다.

    !! 속성이 아니라 자식 <param> 이어야 한다 !!
    ros2_control URDF 파서는 initial_value 를 <param> 자식에서만 읽는다.
    setAttribute 로 넣으면 오류도 경고도 없이 조용히 무시되고, 12개 관절이
    전부 0 으로 생성돼 Go2 가 다리를 편 채 떨어진다. 로그에서 확인하는 법:
        [gz_quadruped_control]:      found initial value: 1.270000
    이 줄이 관절 수만큼 나와야 한다. 0 건이면 시딩이 안 먹은 것이다.
    """
    for existing in interface.getElementsByTagName('param'):
        if existing.getAttribute('name') == 'initial_value':
            for child in list(existing.childNodes):
                existing.removeChild(child)
            existing.appendChild(document.createTextNode(str(value)))
            return
    param = document.createElement('param')
    param.setAttribute('name', 'initial_value')
    param.appendChild(document.createTextNode(str(value)))
    interface.appendChild(param)


# 스폰부터 컨트롤러가 붙을 때까지 관절을 붙잡아 둘 이득.
# 이게 없으면 그 구간(libtorch 로드 10~20초) 동안 관절에 힘이 전혀 안 걸려
# 로봇이 무너지고, 발이 지면을 0.2 m 파고든 채 갇혀 그 뒤 FIXEDSTAND 가
# 관절을 정확히 기립 자세로 보내도 몸통이 0.107 m 에서 안 올라온다.
# 컨트롤러의 stand_kp(80)보다 조금 낮춰 착지 충격을 줄인다.
# 0 이면 시딩하지 않는다(= 원래 동작).
#
# !! 0 이 아닌 값을 넣어 봤지만 답이 아니었다 !!
# 스폰 직후 컨트롤러가 붙기 전까지 관절을 붙잡아 두려고 kp 60 / kd 3 을
# 넣어 봤다. 하드웨어는 실제로 그 값을 썼고(로그에 60.000000 이 12개),
# 로봇이 무너지지 않고 자세를 유지하는 것까지는 됐다. 그런데 순수 P 제어라
# 자중을 버티려면 큰 위치 오차가 필요해서 다리가 그만큼 주저앉는다 —
# 몸통이 0.115 m 에서 안 올라왔다(정상 기립 0.331 m). 이득을 더 키우면
# 착지 순간이 불안정해진다. 그래서 끄고, 대신 실패한 시행을 재시도한다.
_HOLD_KP = 400.0
_HOLD_KD = 10.0


def _seed_initial_joint_positions(document):
    found = set()
    for control in document.getElementsByTagName('ros2_control'):
        for joint in control.getElementsByTagName('joint'):
            name = joint.getAttribute('name')
            if name not in _SPAWN_POSE:
                continue
            # 명령 쪽도 같이 시딩해야 하드웨어가 실제로 그 자세를 붙잡는다.
            # 위치만 주고 kp 가 0 이면 아무 힘도 안 나온다.
            for interface in joint.getElementsByTagName('command_interface'):
                iname = interface.getAttribute('name')
                if iname == 'position':
                    _set_initial_value(document, interface,
                                       _SPAWN_POSE[name])
                elif iname == 'kp' and _HOLD_KP > 0.0:
                    _set_initial_value(document, interface, _HOLD_KP)
                elif iname == 'kd' and _HOLD_KD > 0.0:
                    _set_initial_value(document, interface, _HOLD_KD)
            for interface in joint.getElementsByTagName('state_interface'):
                if interface.getAttribute('name') == 'position':
                    _set_initial_value(document, interface,
                                       _SPAWN_POSE[name])
                    found.add(name)
                    break
    missing = set(_SPAWN_POSE) - found
    if missing:
        raise RuntimeError('관절 누락: ' + ', '.join(sorted(missing)))


def _setup(context, *args, **kwargs):
    ns = 'leg'
    ctl = LaunchConfiguration('controller').perform(context)
    policy = LaunchConfiguration('policy').perform(context)
    world = LaunchConfiguration('world_name').perform(context)
    init_z = LaunchConfiguration('world_init_z').perform(context)

    desc = get_package_share_directory('agconav_description')
    doc = xacro.process_file(
        os.path.join(desc, 'urdf', 'leg', 'leg_rl_with_sensors.urdf.xacro'),
        mappings={'GAZEBO': 'true'})
    _seed_initial_joint_positions(doc)
    robot_description = doc.toxml()

    # !! 높이를 재려면 3D odom 을 따로 붙여야 한다 !!
    # 운용 xacro 의 OdometryPublisher 에는 <dimensions> 가 없어 2D 다 — z 가
    # 항상 0 으로 나온다. 이 실험은 몸통 높이와 등판 높이가 판정 기준이라
    # z 가 필요하다. 운용 파일을 고치는 대신, 여기서 처리된 XML 문자열에만
    # 3D 짜리를 하나 더 끼워 넣는다(운용 조건은 그대로 둔다).
    # tf_topic 을 따로 준 이유: 기본값이면 이 발행기까지 /tf 에 써서 운용
    # 발행기와 같은 프레임을 두 번 내보낸다.
    _ODOM3D = ('<gazebo><plugin'
               ' filename="gz-sim-odometry-publisher-system"'
               ' name="gz::sim::systems::OdometryPublisher">'
               '<odom_frame>odom</odom_frame>'
               '<robot_base_frame>trunk</robot_base_frame>'
               '<odom_publish_frequency>50</odom_publish_frequency>'
               '<dimensions>3</dimensions>'
               '<tf_topic>/bench/odom3d_tf</tf_topic>'
               '<odom_topic>/bench/odom3d</odom_topic>'
               '</plugin></gazebo>')
    robot_description = robot_description.replace('</robot>', _ODOM3D + '</robot>')

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='log',
               parameters=[{'use_sim_time': True,
                            'robot_description': robot_description,
                            'frame_prefix': f'{ns}/',
                            'publish_frequency': 50.0}])

    # create 가 robot_description 토픽의 1회성 대용량 샘플을 놓치면 모델이
    # 영원히 생성되지 않는다. URDF 를 직접 넘겨 DDS 전달 경로를 없앤다.
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', ns, '-string', robot_description,
                            '-x', '0', '-y', '0', '-z', init_z, '-Y', '0'])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='leg_gazebo_bridge', output='log',
                  parameters=[{'use_sim_time': True}],
                  arguments=['/leg/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                             '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                             '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'])

    # 정답 포즈. 위에서 끼워 넣은 3D OdometryPublisher 가 월드 절대좌표로 낸다.
    # GroupAction(/odom 리맵) 밖에 두어야 한다 — 안에 넣으면 같이 리맵된다.
    # (월드의 dynamic_pose/info 를 브리지하는 방법도 해 봤다. 토픽 이름은
    #  맞는데 데이터가 오지 않았다. 이쪽이 확실하다.)
    pose_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='bench_pose_bridge', output='log',
        parameters=[{'use_sim_time': True}],
        arguments=['/bench/odom3d@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    def spawner(names, params=None):
        return Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=list(names) + [
                '--controller-manager', '/controller_manager',
                # libtorch 로드가 오래 걸린다. 넉넉히 준다.
                '--controller-manager-timeout', '300',
                '--switch-timeout', '300',
                '--service-call-timeout', '180'],
            parameters=params or [],
            additional_env={'ROS_HOME': os.path.join(
                os.path.expanduser('~'), '.ros', 'agconav_leg')})

    # !! spawner 를 나눈다 !!
    # 하나로 묶으면 앞 컨트롤러가 실패할 때 뒤가 통째로 날아간다. 실제로
    #   A controller named 'joint_state_broadcaster' was already loaded
    # 에서 spawner 가 죽어 보행 컨트롤러가 로드조차 되지 않은 적이 있다.
    broadcasters = spawner(['joint_state_broadcaster', 'imu_sensor_broadcaster'])
    if ctl == 'guide':
        controllers = spawner(['unitree_guide_controller'])
        walk_cmd, norm_lin = 4, 0.4     # TROTTING 진입, ly 는 [-1,1] 정규화 축
    else:
        # !! FIXEDDOWN 이 깊이 웅크리지 않게 목표 자세를 얕게 덮어쓴다 !!
        # 기본 down_pos 는 calf -2.8 로 정강이를 완전히 접어 배 밑에 깔고
        # 앉는 자세다. 그 자세로 지면에 닿으면 정강이가 몸무게에 눌려 다시
        # 못 일어난다 — 실측: 관절은 정확히 기립 자세(thigh 0.80, calf -1.57)
        # 까지 갔는데 몸통은 0.116 m 에 머물렀다(정상 기립 0.331 m).
        # 그렇다고 기립 자세와 똑같이 두면 이번엔 웅크림이 사라져 "컨트롤러가
        # 관절을 잡기 시작했다"를 몸통 높이로 감지할 수 없다(실제로 감지에
        # 실패해 90초 타임아웃까지 FIXEDDOWN <-> FIXEDSTAND 를 왕복했다).
        # thigh 1.0 / calf -2.0 이면 몸통이 0.25 m 로 내려앉아 감지는 되고,
        # 무릎이 지면에서 0.135 m 떠 있어 되일어설 수 있다.
        #
        # 이건 계측 대상이 아니라 하네스 준비 동작이다. 우리가 재는 것은
        # 경사 등판 성능이라 기립 자세 세부는 영향이 없다.
        # 순서는 gazebo.yaml 의 joints: 그대로 FR, FL, RR, RL x (hip,thigh,calf).
        #
        # !! 파라미터를 dict 로 넘기면 안 된다 !!
        # launch_ros 가 리스트를 파이썬 tuple 로 직렬화해서 spawner 가
        #   yaml.constructor.ConstructorError:
        #   could not determine a constructor for the tag '...python/tuple'
        # 로 죽는다. 컨트롤러가 아예 로드되지 않아 로봇이 시딩된 자세로
        # 가만히 서 있기만 한다(실측: 진출 0.0 m 인데 원인이 안 보였다).
        # YAML 파일로 직접 써서 넘기면 그 경로를 타지 않는다.
        ctrl_params = os.path.join(TEST_DIR, '..', 'configs',
                                   'leg_ctrl_%s.yaml' % (policy or 'none'))
        ctrl_params = os.path.abspath(ctrl_params)
        os.makedirs(os.path.dirname(ctrl_params), exist_ok=True)
        with open(ctrl_params, 'w') as f:
            f.write('rl_quadruped_controller:\n  ros__parameters:\n')
            f.write('    model_folder: "%s"\n' % policy)
            f.write('    down_pos: [%s]\n'
                    % ', '.join('%.3f' % v for v in [0.0, 1.0, -2.0] * 4))
        controllers = spawner(['rl_quadruped_controller'], [ctrl_params])

    relay = Node(executable=os.path.join(TEST_DIR, 'bench_relay.py'),
                 name='bench_relay', output='screen',
                 parameters=[{'use_sim_time': True,
                              'cmd_vel_topic': '/leg/cmd_vel',
                              'output_topic': '/control_input',
                              'walk_command': walk_cmd,
                              'norm_linear': norm_lin,
                              'max_linear': 1.5,
                              'max_angular': 1.0}])

    # !! libtorch 경로를 런치가 직접 넣어야 한다 !!
    # 셸에서 export 해도 ros2 launch 로 띄운 controller_manager 프로세스에는
    # 전달되지 않는 경우가 있다. dlopen error: libc10.so 로 죽는다.
    torch_lib = os.path.join(os.path.expanduser('~'), 'libtorch', 'lib')
    ld = os.environ.get('LD_LIBRARY_PATH', '')

    return [rsp, pose_bridge, GroupAction([
        SetEnvironmentVariable('LD_LIBRARY_PATH',
                               torch_lib + (':' + ld if ld else '')),
        SetRemap('/tf', '/leg/tf'),
        SetRemap('/tf_static', '/leg/tf_static'),
        SetRemap('/odom', '/leg/odom'),
        spawn, bridge,
        # 모델을 무제어로 두면 몇 초 안에 바닥에 눕는다. 추정 시간 대신
        # 실제 완료 이벤트로 즉시 이어 붙인다.
        broadcasters,
        RegisterEventHandler(OnProcessExit(
            target_action=broadcasters, on_exit=[controllers])),
        # !! 중계를 컨트롤러보다 먼저 띄운다 !!
        # 컨트롤러는 활성화되는 순간 PASSIVE 로 들어가며 kp 를 0 으로 만든다
        # (StatePassive::enter). 그 순간에 명령 2 가 이미 도착해 있어야 다음
        # 주기(5 ms)에 FIXEDDOWN 이 kp 를 다시 건다. 컨트롤러 뒤에 띄우면
        # 중계 시작 + DDS 발견까지 수백 ms 가 비어 그 사이에 로봇이 눕는다.
        # 먼저 띄워 두면 컨트롤러가 구독을 만드는 시점에 이쪽 발행자가 이미
        # 발견돼 있어 정합이 즉시 끝난다.
        relay,
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='rl'),
        DeclareLaunchArgument('policy', default_value='robot_lab'),
        DeclareLaunchArgument('world_name', default_value='Seongdong_gu'),
        DeclareLaunchArgument('world_init_z', default_value='0.4'),
        OpaqueFunction(function=_setup),
    ])
