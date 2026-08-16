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
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    ns = LaunchConfiguration('robot_name').perform(context)
    model_folder = LaunchConfiguration('model_folder').perform(context)
    max_lin = LaunchConfiguration('max_linear').perform(context)

    desc = get_package_share_directory('agconav_description')
    robot_description = xacro.process_file(
        os.path.join(desc, 'urdf', 'leg', 'leg_rl_with_sensors.urdf.xacro'),
        mappings={'GAZEBO': 'true'}).toxml()

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='log',
               parameters=[{'use_sim_time': use_sim_time,
                            'robot_description': robot_description,
                            'publish_frequency': 50.0}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', ns, '-topic', 'robot_description',
                            '-x', LaunchConfiguration('world_init_x'),
                            '-y', LaunchConfiguration('world_init_y'),
                            '-z', LaunchConfiguration('world_init_z'),
                            '-Y', LaunchConfiguration('world_init_heading')])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='leg_gazebo_bridge', output='log',
                  parameters=[{'use_sim_time': use_sim_time}],
                  arguments=['/leg/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                             '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                             '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'])

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

    return [GroupAction([
        SetEnvironmentVariable('LD_LIBRARY_PATH',
                               torch_lib + (':' + ld if ld else '')),
        SetRemap('/tf', '/leg/tf'),
        SetRemap('/tf_static', '/leg/tf_static'),
        SetRemap('/odom', '/leg/odom'),
        rsp, spawn, bridge,
        # 하드웨어 초기화가 끝날 즈음 올린다. 바로 부르면 gz 하드웨어가 관절
        # 12개를 올리는 동안 controller_manager 가 서비스 콜백을 못 돌린다.
        TimerAction(period=25.0, actions=[broadcasters]),
        # !! RL 컨트롤러는 기동 폭풍이 지난 뒤에 올린다 !!
        # 이 컨트롤러만 700 MB libtorch 를 dlopen 한다. 전체 스택(Nav2 2벌 +
        # 2,808만 칸 코스트맵 초기화)이 도는 동안 Gazebo 프로세스는 CPU 99.9%
        # 이고, 그 안에서 도는 controller_manager 가 서비스 콜백을 못 돌려
        # spawner 가 list_controllers 대기에서 영영 멈춘다(실측).
        # 브로드캐스터는 가벼워 25초에 올려도 통과한다 — 무거운 것만 미룬다.
        TimerAction(period=120.0, actions=[controllers]),
        # 컨트롤러가 active 된 뒤에 기립 시퀀스를 시작해야 한다.
        # 컨트롤러가 active 된 뒤에 기립 시퀀스를 시작해야 한다.
        TimerAction(period=150.0, actions=[relay]),
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
