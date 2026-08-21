"""실험 1: Go2 를 지정한 월드에 띄우고 지정한 컨트롤러로 걷게 한다.

    ros2 launch test/scripts/leg_terrain.launch.py \
        world:=<sdf> controller:=rl policy:=robot_lab

controller
    rl     -> rl_quadruped_controller (정책은 policy 인자로 고른다)
    guide  -> unitree_guide_controller (Unitree 공식 unitree_guide 이식본)

두 컨트롤러 모두 control_input_msgs/Inputs 로 FSM 을 몰고 cmd_vel 은 받지
않으므로, 시험 스크립트가 /control_input 을 직접 낸다(중계 노드를 안 쓴다).
GUI 는 띄우지 않는다 — 여기서 재는 것은 숫자뿐이다.
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# gazebo.yaml 의 rl_quadruped_controller down_pos 를 관절 이름으로 푼 것.
# 그 순서는 FR, FL, RR, RL 이다(unitree_guide 쪽 목록과 다르니 주의).
# 생성 직후부터 FIXEDDOWN 자세로 두어야 정책이 무너진 자세에서 시작하지 않는다.
DOWN = {
    'FR_hip_joint': 0.01, 'FR_thigh_joint': 1.27, 'FR_calf_joint': -2.8,
    'FL_hip_joint': -0.01, 'FL_thigh_joint': 1.27, 'FL_calf_joint': -2.8,
    'RR_hip_joint': 0.3, 'RR_thigh_joint': 1.31, 'RR_calf_joint': -2.8,
    'RL_hip_joint': -0.3, 'RL_thigh_joint': 1.31, 'RL_calf_joint': -2.8,
}


def _seed(doc):
    """initial_value 는 속성이 아니라 <param> 자식이어야 파서가 읽는다."""
    found = set()
    for ctl in doc.getElementsByTagName('ros2_control'):
        for j in ctl.getElementsByTagName('joint'):
            name = j.getAttribute('name')
            if name not in DOWN:
                continue
            for itf in j.getElementsByTagName('state_interface'):
                if itf.getAttribute('name') != 'position':
                    continue
                p = doc.createElement('param')
                p.setAttribute('name', 'initial_value')
                p.appendChild(doc.createTextNode(str(DOWN[name])))
                itf.appendChild(p)
                found.add(name)
                break
    missing = set(DOWN) - found
    if missing:
        raise RuntimeError('관절 누락: ' + ', '.join(sorted(missing)))


def _setup(context, *a, **k):
    world = LaunchConfiguration('world').perform(context)
    ctl = LaunchConfiguration('controller').perform(context)
    policy = LaunchConfiguration('policy').perform(context)

    # 운용 스택과 같은 모델을 쓴다. go2_description 원본만 띄우면 RL 정책이
    # 기립 후 옆으로 쓰러졌다(z 0.32 -> 0.162). 운용에서 잘 걷는 구성과
    # 조건을 맞춰야 컨트롤러 비교가 의미를 갖는다.
    doc = xacro.process_file(
        os.path.join(get_package_share_directory('agconav_description'),
                     'urdf', 'leg', 'leg_rl_with_sensors.urdf.xacro'),
        mappings={'GAZEBO': 'true'})
    _seed(doc)
    urdf = doc.toxml()
    # !! 포즈는 ROS 토픽으로 읽는다 !!
    # gz topic / gz model 의 텍스트 출력은 position 과 orientation 이 같은 키
    # 이름(x,y,z)을 써서 잘못 읽기 쉽고, 모델이 아예 안 잡히는 경우도 있었다.
    # OdometryPublisher 를 URDF 에 넣고 /odom 을 브리지하면 정답 포즈가
    # 표준 메시지로 나온다.
    odom = ('<gazebo>'
            '<plugin filename="gz-sim-odometry-publisher-system"'
            ' name="gz::sim::systems::OdometryPublisher">'
            '<odom_frame>odom</odom_frame>'
            '<robot_base_frame>trunk</robot_base_frame>'
            '<odom_publish_frequency>50</odom_publish_frequency>'
            '<odom_topic>/odom3d</odom_topic>'
            '<dimensions>3</dimensions>'
            '</plugin></gazebo>')
    # 운용 xacro 에도 OdometryPublisher 가 있지만 dimensions 가 없어 2D 다
    # (z 가 항상 0 으로 나온다). 여기서는 높이가 판정 기준이므로 3D 짜리를
    # 따로 붙이고 토픽을 분리한다.
    urdf = urdf.replace('</robot>', odom + '</robot>')

    if ctl == 'guide':
        names, params = ['unitree_guide_controller'], []
    else:
        names, params = ['rl_quadruped_controller'], [{'model_folder': policy}]

    return [
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'go2', '-string', urdf,
                        '-x', '0', '-y', '0', '-z', '0.35', '-Y', '0']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='log',
             parameters=[{'use_sim_time': True, 'robot_description': urdf}]),
        Node(package='controller_manager', executable='spawner', output='log',
             arguments=['joint_state_broadcaster', 'imu_sensor_broadcaster',
                        '--controller-manager-timeout', '120']),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=names + ['--controller-manager-timeout', '180'],
             parameters=params),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='log',
             arguments=['/odom3d@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('controller', default_value='rl'),
        DeclareLaunchArgument('policy', default_value='robot_lab'),
        # rl_quadruped_controller 는 libtorch 를 링크한다. controller_manager 가
        # Gazebo 프로세스 안에서 도므로 여기서 넣어야 플러그인이 열린다.
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            os.path.expanduser('~/libtorch/lib') + os.pathsep
            + os.environ.get('LD_LIBRARY_PATH', '')),
        OpaqueFunction(function=_setup),
    ])
