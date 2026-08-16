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
from launch.substitutions import LaunchConfiguration
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
        DeclareLaunchArgument(
            "world_file",
            default_value=os.path.join(
                get_package_share_directory("agconav_worlds"), "worlds",
                "Seongdong_gu_aligned", "Seongdong_gu_aligned.world"),
                              description="띄울 .world 절대경로 (비우면 기본 월드)"),
        # 전체 맵 스캔 경로. 간격 3 m 는 실험으로 확정한 값이다 —
        # 자유 공간이 단일 덩어리로 이어지는 가장 싼 간격이고(단일성 99.1%),
        # 이 경로로 만든 지도에서 종단 주행이 191.5 m 완주했다.
        # 근거: agconav_test_worlds/10~12 번 문서.
        DeclareLaunchArgument(
            "drone_path_file",
            default_value=os.path.join(
                get_package_share_directory("agconav_drone"), "config",
                "scan_path_fullmap.yaml"),
                              description="드론 스캔 경로 YAML"),
        # world_file 이 기본 월드에 없는 model:// 을 참조할 때 필요하다.
        # 없으면 Gazebo가 모델을 못 찾아 월드 로드가 실패하고, 월드가 없으니
        # 모듈 A~F가 전부 연쇄로 죽는다.
        DeclareLaunchArgument("model_path", default_value="",
                              description="GZ_SIM_RESOURCE_PATH에 덧붙일 모델 폴더"),
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
        # 스폰 좌표는 agconav_sim 에도 선언돼 있지만, 부모에서 LaunchConfiguration
        # 으로 넘기려면 여기서도 선언돼 있어야 한다(미선언 시 include 시점에 오류).
        DeclareLaunchArgument("wheel_x", default_value="-195.2127"),  # 정렬 월드,
        DeclareLaunchArgument("wheel_y", default_value="73.0167"),
        DeclareLaunchArgument("wheel_z", default_value="6.1766"),
        DeclareLaunchArgument("wheel_yaw", default_value="-0.0503"),
        DeclareLaunchArgument("leg_x", default_value="-195.6963"),
        DeclareLaunchArgument("leg_y", default_value="76.3734"),
        DeclareLaunchArgument("leg_z", default_value="6.1612"),
        DeclareLaunchArgument("leg_yaw", default_value="-0.0767"),
        DeclareLaunchArgument("cruise_speed", default_value="10.0",
                              description="드론 순항 속도 [m/s]"),
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
            "path_file": LaunchConfiguration("drone_path_file"),
            "cruise_speed": LaunchConfiguration("cruise_speed"),
        }.items(),
        condition=IfCondition(enable_drone),
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
            module_d,
            module_e,
            module_f,
        ]
    )

