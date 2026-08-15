"""experiment.launch.py — 테스트 월드에서 드론 스캔을 날리고, 지도·주행성까지 한 번에.

    flight:=teleport   drone_pose_controller + SetEntityPose (리포지토리 현재 방식)
    flight:=velocity   같은 waypoint 를 멀티콥터 속도 제어로 실제 비행

두 모드 모두 `/model/X3/pose` 를 `/tf` 로 브리지한다. agconav_sim 이 하는 것과 같고,
모듈 A 가 점을 map 으로 옮길 때 쓰는 것도 이것이다. 월드의 OdometryPublisher 가
모델의 실제 pose 를 내보내므로 평가의 기준값(ground truth)도 같은 스트림이다.

    module_a:=true   drone_elevation_mapper 를 함께 실행 (기본)
    module_f:=true   terrain_feature_calculator + verdictor(wheel/leg) 까지 실행
                     → 종단 테스트: 비행 → 2.5D 지도 → 주행성 지도

velocity 모드는 중력이 켜진 드론 사본을 쓰는 월드 변형본이 필요하다. 원본 드론
모델은 SetEntityPose 순간이동을 전제로 gravity 0 이라 로터가 기체를 밀지 못한다.

    ros2 launch agconav_test_worlds experiment.launch.py flight:=velocity module_f:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'Seongdong_gu'
WORLD_DIR = 'Seongdong_gu_100x100'
WORLD_DIR_DYNAMIC = 'Seongdong_gu_100x100_dynamic'


def generate_launch_description():
    share = get_package_share_directory('agconav_test_worlds')
    drone_share = get_package_share_directory('agconav_drone')
    trav_share = get_package_share_directory('agconav_traversability')
    worlds_share = get_package_share_directory('agconav_worlds')
    desc_share = get_package_share_directory('agconav_description')
    bringup_share = get_package_share_directory('agconav_bringup')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    flight = LaunchConfiguration('flight')
    headless = LaunchConfiguration('headless')
    bag_output = LaunchConfiguration('bag_output')
    maps_output = LaunchConfiguration('maps_output')
    run_module_a = LaunchConfiguration('module_a')
    run_module_f = LaunchConfiguration('module_f')
    use_rviz = LaunchConfiguration('rviz')
    path_file_arg = LaunchConfiguration('path_file')
    cruise_speed_arg = LaunchConfiguration('cruise_speed_mps')

    is_teleport = PythonExpression(["'", flight, "' == 'teleport'"])
    is_velocity = PythonExpression(["'", flight, "' == 'velocity'"])

    args = [
        DeclareLaunchArgument('flight', default_value='teleport',
                              description='teleport | velocity'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('module_a', default_value='true',
                              description='drone_elevation_mapper 를 함께 실행'),
        DeclareLaunchArgument('module_f', default_value='false',
                              description='모듈 F(주행성)까지 실행 — 종단 테스트'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz 로 스캔·지도·주행성을 함께 본다'),
        DeclareLaunchArgument('path_file', default_value='path_100x100.yaml',
                              description='share/config/ 아래 웨이포인트 yaml 파일명 '
                                          '(5m AGL 실험용 path_100x100_5m_{2m,3m,4m}.yaml 등)'),
        DeclareLaunchArgument('cruise_speed_mps', default_value='8.0',
                              description='velocity 모드(velocity_path_follower) 순항 속도. '
                                          'teleport 모드는 drone_path_player.yaml의 값을 그대로 씀 '
                                          '(기존 동작 유지, 이 인자의 영향을 받지 않음).'),
        DeclareLaunchArgument(
            'bag_output',
            default_value=[os.path.join(os.getcwd(), 'bags', 'exp_'), flight]),
        DeclareLaunchArgument(
            'maps_output',
            default_value=[os.path.join(os.getcwd(), 'maps_test', 'exp_'), flight]),
    ]

    def world_for(name):
        return os.path.join(share, 'worlds', name, name + '.world')

    gz_dirs = [os.path.join(desc_share, 'models'), os.path.join(worlds_share, 'models'),
               os.path.join(share, 'models')]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        gz_dirs.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    set_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(gz_dirs))

    # {teleport, velocity} x {gui, headless} 네 조합.
    gz = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world_for(name), extra]}.items(),
        condition=IfCondition(PythonExpression(
            ["'", flight, "' == '", mode, "' and ",
             'not ' if gui else '', "'", headless, "'.lower() in ('true','1')"])))
        for mode, name in (('teleport', WORLD_DIR), ('velocity', WORLD_DIR_DYNAMIC))
        for extra, gui in ((' -r', True), (' -r -s', False))]

    clock = Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='exp_clock_bridge', output='log',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    sensors = Node(package='ros_gz_bridge', executable='parameter_bridge',
                   name='exp_sensor_bridge', output='log',
                   parameters=[{'use_sim_time': True}],
                   arguments=[
                       '/drone/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                       '/drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                   remappings=[('/drone/points/points', '/drone/points')])

    # agconav_sim 과 동일: 월드의 OdometryPublisher 가 map -> drone/base_link 로
    # 설정돼 있고, 그 pose 를 그대로 /tf 에 브리지한다.
    drone_tf = Node(package='ros_gz_bridge', executable='parameter_bridge',
                    name='exp_drone_tf_bridge', output='log',
                    arguments=['/model/X3/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
                    remappings=[('/model/X3/pose', '/tf')])

    lidar_tf = Node(package='tf2_ros', executable='static_transform_publisher',
                    name='drone_os1_lidar_static_tf', output='log',
                    arguments=['--x', '0', '--y', '0', '--z', '-0.175406',
                               '--roll', '0', '--pitch', '1.5708', '--yaw', '0',
                               '--frame-id', 'drone/base_link',
                               '--child-frame-id', 'drone/os1_lidar'],
                    parameters=[{'use_sim_time': True}])

    # ---- teleport 비행 -------------------------------------------------------
    set_pose_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='exp_set_pose_bridge', output='log', condition=IfCondition(is_teleport),
        arguments=['/world/%s/set_pose@ros_gz_interfaces/srv/SetEntityPose' % WORLD_NAME])

    pose_controller = Node(
        package='agconav_drone', executable='drone_pose_controller',
        name='drone_pose_controller', output='screen', condition=IfCondition(is_teleport),
        parameters=[os.path.join(drone_share, 'config', 'drone_pose_controller.yaml'),
                    {'world_name': WORLD_NAME, 'entity_name': 'X3', 'use_sim_time': True}])

    path_player = Node(
        package='agconav_drone', executable='drone_path_player',
        name='drone_path_player', output='screen', condition=IfCondition(is_teleport),
        parameters=[os.path.join(drone_share, 'config', 'drone_path_player.yaml'),
                    {'path_file': PathJoinSubstitution([share, 'config', path_file_arg]),
                     'use_sim_time': True}])

    # ---- velocity 비행 -------------------------------------------------------
    cmd_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='exp_cmd_bridge', output='log', condition=IfCondition(is_velocity),
        arguments=['/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean'],
        remappings=[('/X3/gazebo/command/twist', '/drone/cmd_vel'),
                    ('/X3/enable', '/drone/enable')])

    follower = Node(
        package='agconav_test_worlds', executable='velocity_path_follower.py',
        name='velocity_path_follower', output='screen', condition=IfCondition(is_velocity),
        parameters=[{'path_file': PathJoinSubstitution([share, 'config', path_file_arg]),
                     'cruise_speed_mps': ParameterValue(cruise_speed_arg, value_type=float),
                     'use_sim_time': True}])

    # ---- 모듈 A (2.5D 지도) --------------------------------------------------
    mapper = Node(
        package='agconav_drone', executable='drone_elevation_mapper',
        name='drone_elevation_mapper', output='screen', condition=IfCondition(run_module_a),
        parameters=[os.path.join(drone_share, 'config', 'drone_elevation_mapper.yaml'),
                    {'use_sim_time': True}])

    # ---- 모듈 F (주행성) -----------------------------------------------------
    # 모듈 F 의 계산 트리거는 /drone/elevation_map_status 이고, 그것은 모듈 A 쪽
    # elevation_map_saver 가 발행한다. saver 는 저장 디렉터리가 이미 있으면 실패해
    # 트리거를 내지 않으므로 maps_output 을 실행마다 비운 경로로 준다.
    map_saver = Node(
        package='agconav_drone', executable='elevation_map_saver',
        name='elevation_map_saver', output='screen', condition=IfCondition(run_module_f),
        parameters=[os.path.join(drone_share, 'config', 'elevation_map_saver.yaml'),
                    {'output_directory': maps_output, 'use_sim_time': True}])

    feature_calc = Node(
        package='agconav_traversability', executable='terrain_feature_calculator',
        name='terrain_feature_calculator', output='screen', condition=IfCondition(run_module_f),
        parameters=[os.path.join(trav_share, 'config', 'terrain_feature_calculator.yaml'),
                    {'use_sim_time': True}])

    # verdictor 두 개는 노드 이름을 명시해야 한다. yaml 키가
    # traversability_verdictor_wheel / _leg 라서, 이름이 없으면 둘 다 기본값(wheel)
    # 으로 떠서 leg 지도가 나오지 않는다.
    verdictors = [
        Node(package='agconav_traversability', executable='traversability_verdictor',
             name='traversability_verdictor_%s' % robot, output='screen',
             condition=IfCondition(run_module_f),
             parameters=[os.path.join(trav_share, 'config', 'traversability_%s.yaml' % robot),
                         {'use_sim_time': True}])
        for robot in ('wheel', 'leg')]

    # ---- 녹화 ---------------------------------------------------------------
    # 필요한 것만 담는다. 전체 토픽을 담은 velocity 실행이 200초에 54 GB(초당
    # 270 MB, teleport 의 30배)를 써 비행이 끝나기 전에 디스크를 채웠다. 원인은
    # 이전 실행에서 죽지 않고 남은 sensor 브리지들의 중복 발행이었다.
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_output,
             '--max-bag-size', '2000000000',
             '/drone/points', '/drone/imu', '/tf', '/tf_static',
             '/drone/elevation_map', '/drone/path_status',
             '/terrain/features', '/wheel/nav_map', '/leg/nav_map',
             '/wheel/nav_map_status', '/leg/nav_map_status'],
        output='log')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(share, 'rviz', 'experiment.rviz')],
        parameters=[{'use_sim_time': True}])

    wait = ExecuteProcess(
        cmd=[os.path.join(bringup_share, 'scripts', 'wait_for_world.sh'), WORLD_NAME, '180'],
        output='screen', name='wait_for_world')

    def _go(event, context):        # noqa: ARG001
        if event.returncode != 0:
            return [Shutdown(reason='world load timeout')]
        return [recorder, pose_controller, path_player, follower,
                mapper, map_saver, feature_calc] + verdictors

    return LaunchDescription(args + [
        set_path, *gz, clock, sensors, drone_tf, lidar_tf,
        set_pose_bridge, cmd_bridge, rviz, wait,
        RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=_go)),
    ])
