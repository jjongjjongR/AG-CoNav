"""AG-CoNav 전체 통합 실행 — 시뮬레이션 + 모듈 A~F.

agconav_sim.launch.py(월드·로봇 3종·모듈 B) 위에 나머지 모듈을 얹는다.
각 모듈 패키지는 그대로 두고 여기서 include만 한다(모듈 코드 무수정 원칙).

데이터 흐름 (topics.md 기준):

    모듈 A  /drone/points        -> /drone/elevation_map (+status)
      |
    모듈 F  /drone/elevation_map -> /wheel/nav_map, /leg/nav_map (+status)
      |
    모듈 C  /X/nav_map + Nav2    -> /X/navigation_status, /X/points_filtered
      |
    모듈 D  /X/points + status   -> /X/elevation_map (+status)
      |
    모듈 E  drone+wheel+leg      -> /merged/elevation_map

모듈 C의 navigation_complete_node는 Nav2의 navigate_to_pose 액션 상태를 보므로
use_nav2:=true 여야 /X/navigation_status가 나온다. 그게 없으면 모듈 D가
지도를 발행하지 않고, 이어서 모듈 E도 트리거되지 않는다.

사용법:
    ros2 launch agconav_bringup agconav_all.launch.py
    ros2 launch agconav_bringup agconav_all.launch.py use_nav2:=true
    ros2 launch agconav_bringup agconav_all.launch.py enable_mapping:=false
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")
    enable_drone = LaunchConfiguration("enable_drone")
    enable_mapping = LaunchConfiguration("enable_mapping")
    enable_fusion = LaunchConfiguration("enable_fusion")
    enable_traversability = LaunchConfiguration("enable_traversability")

    bringup_share = get_package_share_directory("agconav_bringup")

    def _launch(package, filename):
        return os.path.join(get_package_share_directory(package), "launch", filename)

    declares = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # 축소 테스트 월드에서 전체 스택(A~F)을 돌릴 때 쓴다. 비우면 기본 월드.
        # <world name>은 "Seongdong_gu"를 유지해야 clearpath spawn과 set_pose
        # 서비스 경로가 그대로 맞는다.
        DeclareLaunchArgument("world_file", default_value="",
                              description="띄울 .world 절대경로 (비우면 기본 월드)"),
        DeclareLaunchArgument("drone_path_file", default_value="",
                              description="드론 경로 YAML (비우면 전체 월드용 기본 경로)"),
        # flight:=velocity 로 방식 4 월드를 띄울 때는 반드시 함께 줘야 한다.
        #   model_path:=$PWD/src/agconav_test_worlds/models
        # 없으면 Gazebo가 model://agconav_drone_dynamic 을 못 찾아 월드 로드가
        # 실패하고, 월드가 없으니 모듈 A~F가 전부 연쇄로 죽는다.
        DeclareLaunchArgument("model_path", default_value="",
                              description="GZ_SIM_RESOURCE_PATH에 덧붙일 모델 폴더"),
        # 드론을 어떻게 움직일지. 모듈 A~F 코드는 어느 쪽이든 동일하게 돈다.
        #
        # 측정 결과 velocity가 확실히 낫다(RESULTS.md 5·9절, 같은 경로·같은 모듈):
        #                     teleport   velocity
        #   측정 커버리지        78.7%      98.9%
        #   높이 오차 sigma    0.1175 m   0.0839 m
        #   단차 중앙값        0.1287 m   0.0790 m
        #   wheel 주행가능       13.8%      41.8%
        #   leg 주행가능         41.9%      76.5%
        #
        # 이유는 시야 기하다. 순간이동은 고도 84.00~84.01 m에 자세가 항상 수평이라
        # 늘 같은 연직 시선만 쓰는 반면, 실제 비행은 기울고 오르내리며 근거리
        # 반사를 더 얻는다(84 m 미만 반사 38.7% -> 53.2%). 오차는 센서 거리에
        # 비례하므로 그만큼 정확해진다.
        #
        # velocity는 중력이 켜진 드론 모델과 천장 없는 스폰이 필요해서
        # worlds/Seongdong_gu_100x100_dynamic 을 world_file로 줘야 한다.
        # 승인된 Seongdong_gu_100x100 은 드론 스폰 위에 건물 mesh 천장(4.8949 m)이
        # 있어 velocity로 띄우면 드론이 4.7299 m에서 눌린다.
        DeclareLaunchArgument(
            "flight", default_value="teleport", choices=["teleport", "velocity"],
            description="teleport=SetEntityPose 순간이동, velocity=실제 추력 비행"),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Gazebo GUI 없이 서버만 실행(-s). 장시간 자동 검증용."),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="RViz2를 통합 설정(rviz/agconav.rviz)으로 함께 띄운다."),
        DeclareLaunchArgument(
            "use_nav2", default_value="true",
            description="Nav2 실행(기본 켬). 모듈 C의 navigation_status가 Nav2의 "
                        "navigate_to_pose 액션 상태에서 나오므로, 끄면 모듈 D가 "
                        "지도를 발행하지 않고 모듈 E도 트리거되지 않는다."),
        DeclareLaunchArgument("enable_drone", default_value="true",
                              description="모듈 A (드론 지도 생성)"),
        DeclareLaunchArgument("enable_mapping", default_value="true",
                              description="모듈 D (지상 지도 누적)"),
        DeclareLaunchArgument("enable_fusion", default_value="true",
                              description="모듈 E (지도 병합)"),
        DeclareLaunchArgument("enable_traversability", default_value="true",
                              description="모듈 F (주행성 분석)"),
    ]

    # ── 기반: 월드 + 로봇 3종 + 모듈 B ────────────────────────────────
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "agconav_sim.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_nav2": use_nav2,
            "headless": LaunchConfiguration("headless"),
            # 축소 테스트 월드에서 전체 스택을 돌릴 때 쓴다. 빈 값이면 기본 월드.
            "world_file": LaunchConfiguration("world_file"),
            "model_path": LaunchConfiguration("model_path"),
            "wheel_x": LaunchConfiguration("wheel_x"),
            "wheel_y": LaunchConfiguration("wheel_y"),
            "wheel_z": LaunchConfiguration("wheel_z"),
            "wheel_yaw": LaunchConfiguration("wheel_yaw"),
            "leg_x": LaunchConfiguration("leg_x"),
            "leg_y": LaunchConfiguration("leg_y"),
            "leg_z": LaunchConfiguration("leg_z"),
            "leg_yaw": LaunchConfiguration("leg_yaw"),
        }.items(),
    )

    # ── RViz2 (통합 시각화) ───────────────────────────────────────────
    # 로봇 3종의 점군·TF, 모듈 F의 주행 가능 맵, 모듈 C의 지면 제거 결과,
    # Nav2 코스트맵·경로를 한 창에서 본다. 프레임은 전부 접두어 붙은 이름이라
    # Go2 원본 설정(unitree_go2_sim/rviz)은 맞지 않는다.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bringup_share, "rviz", "agconav.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # ── 모듈 A: 드론 지도 생성 ────────────────────────────────────────
    # launch_gazebo:=false — Gazebo와 clock 브리지는 시뮬 쪽에서 이미 띄운다.
    module_a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch("agconav_drone", "drone_sim_test.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "launch_gazebo": "false",
            # agconav_sim.launch.py의 drone_tf_bridge가 월드 OdometryPublisher의
            # /model/X3/pose를 /tf로 이미 내보낸다. 여기서 drone_path_player까지
            # 같은 map->drone/base_link를 발행하면 한 관계에 발행자가 둘이 되어
            # README 3.1을 깬다. 실측: 같은 타임스탬프에 값이 둘 들어오고 차이가
            # 평균 0.785 m, 최대 0.800 m(= 8 m/s ÷ 10 Hz, 한 프레임 이동량).
            # 명령 pose와 실제 도착 pose가 섞여 모듈 A의 TF 조회가 흔들린다.
            "publish_tf": "false",
            "path_file": LaunchConfiguration("drone_path_file"),
            # teleport일 때만 drone_path_player/drone_pose_controller가 뜬다.
            "flight": LaunchConfiguration("flight"),
        }.items(),
        condition=IfCondition(enable_drone),
    )

    # ── 방식 4: 실제 추력 비행 (flight:=velocity) ──────────────────────
    is_velocity = IfCondition(
        PythonExpression(["'", LaunchConfiguration("flight"), "' == 'velocity'"]))

    # MulticopterVelocityControl은 enableSubTopic으로 True를 먼저 받아야 twist를
    # 받아들인다. /drone/cmd_vel 브리지는 agconav_sim에 이미 있지만 enable은 없어서
    # 이걸 안 띄우면 twist를 아무리 보내도 로터가 돌지 않는다.
    drone_enable_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="drone_enable_bridge",
        output="screen",
        arguments=["/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean"],
        remappings=[("/X3/enable", "/drone/enable")],
        condition=is_velocity,
    )

    # 경로는 teleport와 같은 파일을 쓴다(같은 고도·속도·줄 간격). 다른 것은
    # "어떻게 그 선을 따라가느냐"뿐이다.
    velocity_follower = Node(
        package="agconav_test_worlds",
        executable="velocity_path_follower.py",
        name="velocity_path_follower",
        output="screen",
        parameters=[{
            "path_file": LaunchConfiguration("drone_path_file"),
            "use_sim_time": use_sim_time,
        }],
        condition=is_velocity,
    )

    # ── 모듈 C: 지면 분할 + 이동 완료 신호 ────────────────────────────
    # agconav_sim.launch.py의 _nav2_for가 이미 로봇별로 두 노드를 띄운다
    # (use_nav2 조건부). 여기서 또 띄우면 같은 이름 노드가 중복되므로
    # enable_navigation 인자는 use_nav2로 대체한다.

    # ── 모듈 D: 지상 지도 누적 (로봇별) ───────────────────────────────
    module_d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch("agconav_ground_mapping", "ground_elevation_mapping.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(enable_mapping),
    )

    # ── 모듈 E: 지도 병합 (시스템 1개) ────────────────────────────────
    module_e = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch("agconav_map_fusion", "elevation_map_merge.launch.py")),
        condition=IfCondition(enable_fusion),
    )

    # ── 모듈 F: 주행성 분석 (시스템 1개) ──────────────────────────────
    module_f = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch("agconav_traversability", "traversability.launch.py")),
        condition=IfCondition(enable_traversability),
    )

    return LaunchDescription(
        declares + [
            simulation,
            rviz,
            module_a,
            drone_enable_bridge,
            velocity_follower,
            module_d,
            module_e,
            module_f,
        ]
    )

