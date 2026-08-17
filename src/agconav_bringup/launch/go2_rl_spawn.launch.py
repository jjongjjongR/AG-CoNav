"""Go2(leg)를 **RL 컨트롤러**로 스폰한다. go2_spawn.launch.py(CHAMP)의 대체본.

왜 바꿨나: `11. 컨트롤러 실험 — 경사와 속도.md` 결과.
    평지 속도  RL 1.11 m/s  vs  CHAMP 0.162 m/s   (6.9배)
    경사 5도   RL 0.96      vs  CHAMP 0.033       (29배)
    경사 10도  RL 1.54      vs  CHAMP 0.065       (24배)
    안정 최대  RL 1.045 m/s vs  CHAMP 0.266 m/s   (3.9배)
    등판 성공은 RL 뿐(한계 10~15도).

CHAMP 판과 다른 점만 적는다.
  1. 모델: agconav_description/urdf/leg/leg_rl_with_sensors.urdf.xacro
     (go2_description 판 Go2 + 우리 센서 오버레이. 베이스 링크가 base_link 가
      아니라 trunk 라 오버레이 부착점이 다르다.)
  2. 컨트롤러: rl_quadruped_controller + joint_state_broadcaster +
     imu_sensor_broadcaster. leg_pd_controller 는 **띄우지 않는다** — go2 설정에
     그 컨트롤러의 파라미터 절이 없어서 올리면 인터페이스를 못 내보내고
     Gazebo 까지 죽는다(exit 134). go2 는 하드웨어 kp/kd 에 직접 쓴다.
  3. 명령 경로: Nav2 /leg/cmd_vel -> cmd_vel_to_control_input -> /control_input
     (RL 컨트롤러는 조이스틱 축 형식 Inputs 를 받는다. 그 노드가 FSM 기립
      시퀀스도 몰아 주고 명령을 50 Hz 로 끊김 없이 유지한다.)
  4. CHAMP 의 EKF 두 개(base_to_footprint / footprint_to_odom)는 없다.
     RL 컨트롤러는 자체 상태추정을 쓰고, map->odom 은 모듈 B 가 낸다.

TF 격리는 CHAMP 판과 같은 이유로 유지한다 — leg 스택 내부는 접두어 없는
프레임을 쓰므로 /leg/tf 로 가두고, 전역 /tf 에는 tf_prefix_relay 가 붙인다.
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


# gazebo.yaml 의 **rl_quadruped_controller** down_pos 를 관절 이름으로 풀어 쓴 것.
# 그 값이 곧 컨트롤러의 FIXEDDOWN 목표라, 같은 자세로 생성해 두면 생성 직후부터
# FIXEDDOWN 까지 자세가 끊기지 않는다.
#
# !! gazebo.yaml 안에 joints: 목록이 컨트롤러마다 따로 있고 순서가 다르다 !!
#   unitree_guide_controller.joints = FL, FR, RL, RR
#   rl_quadruped_controller.joints  = FR, FL, RR, RL   <- 우리가 쓰는 쪽
#   gazebo.xacro 의 ros2_control    = FR, FL, RR, RL
# down_pos 는 각 컨트롤러의 joints: 순서로 해석해야 한다. unitree_guide 쪽
# 목록을 보고 RL 의 down_pos 를 읽으면 좌우가 뒤바뀌어 **hip 부호 4개가 전부
# 반대로** 나온다. 실제로 그렇게 넣었더니 뒷다리가 반대로 벌어진 채 생성돼,
# RL 모드로 넘어가는 순간 로봇이 제자리에서 도는 증상이 났다.
# 값을 바꿀 일이 있으면 아래로 확인할 것:
#   python3 -c "import yaml;p=yaml.safe_load(open(
#     'src/quadruped_ros2_control/descriptions/unitree/go2_description/config/gazebo.yaml'
#   ))['rl_quadruped_controller']['ros__parameters'];print(list(zip(p['joints'],p['down_pos'])))"
_INITIAL_DOWN_POSITIONS = {
    'FR_hip_joint': 0.01, 'FR_thigh_joint': 1.27, 'FR_calf_joint': -2.8,
    'FL_hip_joint': -0.01, 'FL_thigh_joint': 1.27, 'FL_calf_joint': -2.8,
    'RR_hip_joint': 0.3, 'RR_thigh_joint': 1.31, 'RR_calf_joint': -2.8,
    'RL_hip_joint': -0.3, 'RL_thigh_joint': 1.31, 'RL_calf_joint': -2.8,
}


def _set_initial_value(document, interface, value):
    """Write <param name="initial_value">value</param> into a state_interface.

    !! 속성이 아니라 자식 <param> 이어야 한다 !!
    ros2_control 의 URDF 파서는 initial_value 를 `<param>` 자식에서만 읽는다
    (ur_description/urdf/inc/ur_joint_control.xacro 가 표준 예시다).
    `interface.setAttribute('initial_value', ...)` 로 넣으면 파서가 조용히
    무시한다 -- 오류도 경고도 없다. 그러면 12개 관절이 전부 0 으로 생성돼
    Go2 가 다리를 쭉 편 채 떨어지고, libtorch 가 로드되는 동안 그대로
    주저앉는다. 로그에서 확인하는 법: gz_quadruped_hardware 가 관절마다
        [gz_quadruped_control]:      found initial value: 1.270000
    을 찍는다. 이 줄이 0 건이면 시딩이 안 먹은 것이다.
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


def _seed_initial_joint_positions(document):
    """Seed Gazebo joints before the heavyweight RL controller is loaded."""
    found = set()
    for control in document.getElementsByTagName('ros2_control'):
        for joint in control.getElementsByTagName('joint'):
            name = joint.getAttribute('name')
            if name not in _INITIAL_DOWN_POSITIONS:
                continue
            for interface in joint.getElementsByTagName('state_interface'):
                if interface.getAttribute('name') == 'position':
                    _set_initial_value(
                        document, interface, _INITIAL_DOWN_POSITIONS[name])
                    found.add(name)
                    break
    missing = set(_INITIAL_DOWN_POSITIONS) - found
    if missing:
        raise RuntimeError(
            'Go2 position state interfaces missing: ' + ', '.join(sorted(missing)))


def _setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    ns = LaunchConfiguration('robot_name').perform(context)
    model_folder = LaunchConfiguration('model_folder').perform(context)
    max_lin = LaunchConfiguration('max_linear').perform(context)

    desc = get_package_share_directory('agconav_description')
    robot_document = xacro.process_file(
        os.path.join(desc, 'urdf', 'leg', 'leg_rl_with_sensors.urdf.xacro'),
        mappings={'GAZEBO': 'true'})
    # With no initial positions Gazebo creates all 12 joints at zero.  The
    # model then collapses while libtorch loads, leaving visibly twisted legs
    # before FIXEDDOWN can recover it.  Spawn directly in that stable pose.
    _seed_initial_joint_positions(robot_document)
    robot_description = robot_document.toxml()

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='log',
               parameters=[{'use_sim_time': use_sim_time,
                            'robot_description': robot_description,
                            'frame_prefix': f'{ns}/',
                            'publish_frequency': 50.0}])

    # create가 robot_description 토픽의 1회성 대용량 샘플을 놓치면 모델이
    # 영원히 생성되지 않는다. 이미 여기서 만든 34 KiB URDF를 직접 넘겨 DDS
    # 전달 경로 자체를 없앤다(리눅스 ARG_MAX보다 충분히 작다).
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', ns, '-string', robot_description,
                            '-x', LaunchConfiguration('world_init_x'),
                            '-y', LaunchConfiguration('world_init_y'),
                            '-z', LaunchConfiguration('world_init_z'),
                            '-Y', LaunchConfiguration('world_init_heading')])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='leg_gazebo_bridge', output='log',
                  parameters=[{'use_sim_time': use_sim_time}],
                  arguments=['/leg/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                             '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                             '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'])

    def spawner(names, params=None):
        return Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=list(names) + [
                '--controller-manager', '/controller_manager',
                # CHAMP 판과 같은 이유로 넉넉히 준다. 전체 스택을 함께 띄우면
                # leg 의 controller_manager 가 190초 뒤에 뜬 실측이 있다.
                '--controller-manager-timeout', '300',
                '--switch-timeout', '300',
                '--service-call-timeout', '180'],
            parameters=params or [],
            # spawner 들이 공유하는 락 때문에 wheel 쪽이 굶는 문제를 피한다.
            additional_env={'ROS_HOME': os.path.join(
                os.path.expanduser('~'), '.ros', 'agconav_leg')})

    # !! spawner 를 나눈다 !!
    # 하나로 묶으면 앞의 컨트롤러에서 실패할 때 뒤가 통째로 날아간다. 실제로
    #   A controller named 'joint_state_broadcaster' was already loaded
    # (gz 하드웨어가 먼저 올려 두는 경우가 있다)에서 spawner 가 죽어
    # rl_quadruped_controller 는 로드조차 되지 않았다. 서로 독립시킨다.
    broadcasters = spawner(['joint_state_broadcaster', 'imu_sensor_broadcaster'])
    controllers = spawner(['rl_quadruped_controller'],
                          [{'model_folder': model_folder}])

    relay = Node(package='agconav_navigation',
                 executable='cmd_vel_to_control_input',
                 name='cmd_vel_to_control_input', output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'cmd_vel_topic': '/%s/cmd_vel' % ns,
                              'output_topic': '/control_input',
                              'max_linear': float(max_lin),
                              'max_angular': 1.0}])

    # !! libtorch 경로를 런치가 직접 넣어야 한다 !!
    # rl_quadruped_controller 는 libtorch(C++)를 링크한다. LD_LIBRARY_PATH 에
    # libtorch/lib 이 없으면 controller_manager 가 플러그인을 못 연다:
    #   dlopen error: libc10.so: cannot open shared object file
    #   -> Failed loading controller rl_quadruped_controller
    # 셸에서 export 해도 ros2 launch 로 띄운 controller_manager 프로세스에는
    # 전달되지 않는 경우가 있어(실제로 종단 실행에서 이렇게 죽었다) 런치에서
    # 명시적으로 설정한다.
    torch_lib = os.path.join(os.path.expanduser('~'), 'libtorch', 'lib')
    ld = os.environ.get('LD_LIBRARY_PATH', '')

    # RSP는 처음부터 접두어가 붙은 프레임을 전역 TF에 낸다. 사설 TF를
    # 리레이하면 정적 조인트 묶음이 누락될 때 센서 트리 전체가 끊어질 수 있다.
    # 나머지 Gazebo/컨트롤러 토픽만 기존처럼 사설 TF에 격리한다.
    return [rsp, GroupAction([
        SetEnvironmentVariable('LD_LIBRARY_PATH',
                               torch_lib + (':' + ld if ld else '')),
        SetRemap('/tf', '/leg/tf'),
        SetRemap('/tf_static', '/leg/tf_static'),
        SetRemap('/odom', '/leg/odom'),
        spawn, bridge,
        # 모델을 무제어로 두면 몇 초 안에 바닥에 눕고 calf 관절이 한계에 걸린다.
        # 추정 시간 대신 실제 완료 이벤트로 즉시 이어 붙인다.
        broadcasters,
        RegisterEventHandler(OnProcessExit(
            target_action=broadcasters, on_exit=[controllers])),
        RegisterEventHandler(OnProcessExit(
            target_action=controllers, on_exit=[relay])),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='leg'),
        DeclareLaunchArgument('world_init_x', default_value='0.0'),
        DeclareLaunchArgument('world_init_y', default_value='0.0'),
        DeclareLaunchArgument('world_init_z', default_value='0.4'),
        DeclareLaunchArgument('world_init_heading', default_value='0.0'),
        # 7번 실험에서 확정한 정책과 안정 최대 속도
        DeclareLaunchArgument('model_folder', default_value='robot_lab'),
        DeclareLaunchArgument('max_linear', default_value='1.0'),
        OpaqueFunction(function=_setup),
    ])
