"""Module B 지상 위치추정 — 로봇 공통 EKF + navsat launch.

wheel·leg 동일 구조. 네임스페이스(PushRosNamespace)로 분리하고,
같은 config(ekf_node.yaml, navsat_transform_node.yaml)를 공유한다.

[TF 발행]  map → X/odom  → 전역 /tf 에 직접 발행 (프레임 이름이 이미 X/* 접두어)
[Topic]    /{ns}/odom (nav_msgs/Odometry) — EKF 필터 출력

사용법 (agconav_sim.launch.py에서 include):
  ros2 launch agconav_localization localization.launch.py \\
      namespace:=wheel odom_topic:=/wheel/platform/odom
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def _launch_setup(context, *args, **kwargs):
    """OpaqueFunction — resolve namespace at launch time for SetRemap dst."""
    from launch.substitutions import LaunchConfiguration

    ns = LaunchConfiguration('namespace').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() in ('true', '1', 'yes')

    pkg_dir = get_package_share_directory('agconav_localization')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf_node.yaml')
    navsat_config = os.path.join(pkg_dir, 'config', 'navsat_transform_node.yaml')

    # 모듈 B는 접두어 붙은 프레임(X/odom, X/base_link)으로 동작하므로 전역 /tf를
    # 직접 읽고 쓴다. 사설 /{ns}/tf로 격리하면 두 가지가 깨진다.
    #   1) 사설 트리에는 CHAMP/clearpath의 접두어 없는 프레임(odom, base_link)만
    #      있어 EKF가 X/odom -> X/base_link 조회에 실패한다.
    #   2) EKF가 발행한 map -> X/odom을 tf_prefix_relay가 다시 접두어를 붙여
    #      X/X/odom으로 만든다(relay는 접두어 없는 트리를 전제로 한다).
    tf_topic = '/tf'
    tf_static_topic = '/tf_static'

    # ── EKF Node ──────────────────────────────────────────────────
    # world_frame=map → map→odom TF 발행 (README §3.1).
    # odom0는 실제 시뮬 토픽(절대 경로)으로 override.
    # 나머지 입력(imu0=imu, odom1=odometry/gps)은 상대 이름 →
    # PushRosNamespace로 /{ns}/imu, /{ns}/odometry/gps로 해결.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            ekf_config,
            {
                'use_sim_time': use_sim_time,
                'odom0': odom_topic,
                'map_frame': 'map',
                'odom_frame': f'{ns}/odom',
                'base_link_frame': f'{ns}/base_link',
                'world_frame': 'map',
            },
        ],
        # odometry/filtered is NOT remapped here (Gap #3: keep as odometry/filtered)
    )

    # ── Navsat Transform Node ─────────────────────────────────────
    # GPS(WGS84) → map 좌표 변환.
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            navsat_config,
            {
                'use_sim_time': use_sim_time,
                'base_link_frame': f'{ns}/base_link',
            },
        ],
        remappings=[
            # gps/fix → gps (계약 토픽 /X/gps, topics.md 센서 표 기준).
            # 모듈 B 문서에는 /X/gps/fix로 적혀 있으나 센서 표가 기준이다.
            # 이름이 어긋나면 발행자가 없어 navsat이 fix를 영영 못 받는다.
            ('gps/fix', 'gps'),
            # output odometry/gps → gps/odom (Gap #2)
            ('odometry/gps', 'gps/odom'),
        ],
    )

    # ── Yaw Consistency Checker ───────────────────────────────────
    # odometry yaw와 IMU yaw 비교 (Gap #4)
    yaw_checker_config = os.path.join(pkg_dir, 'config', 'yaw_consistency_checker.yaml')
    yaw_checker_node = Node(
        package='agconav_localization',
        executable='yaw_consistency_checker',
        name='yaw_consistency_checker',
        output='screen',
        parameters=[
            yaw_checker_config,
            {
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic, # raw odom topic
            },
        ],
        # output is /yaw_check_status → /{ns}/yaw_check_status (via namespace)
    )

    # ── GroupAction: 네임스페이스 + 사설 TF ────────────────────────
    # PushRosNamespace: 상대 토픽을 /{ns}/... 으로 해결
    # SetRemap: 절대 /tf 유지 (프레임이 이미 접두어를 가져 격리 불필요)
    localization_group = GroupAction([
        PushRosNamespace(ns),
        SetRemap(src='/tf', dst=tf_topic),
        SetRemap(src='/tf_static', dst=tf_static_topic),
        ekf_node,
        navsat_node,
        yaw_checker_node,
    ])

    return [localization_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            description='Robot namespace: "wheel" or "leg"'),

        DeclareLaunchArgument(
            'odom_topic',
            description='Absolute odometry topic for EKF odom0 '
                        '(e.g. /wheel/odom, /leg/odom)'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        OpaqueFunction(function=_launch_setup),
    ])
