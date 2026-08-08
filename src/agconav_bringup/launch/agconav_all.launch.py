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
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Gazebo GUI 없이 서버만 실행(-s). 장시간 자동 검증용."),
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
        }.items(),
    )

    # ── 모듈 A: 드론 지도 생성 ────────────────────────────────────────
    # launch_gazebo:=false — Gazebo와 clock 브리지는 시뮬 쪽에서 이미 띄운다.
    module_a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch("agconav_drone", "drone_sim_test.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "launch_gazebo": "false",
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
            module_a,
            module_d,
            module_e,
            module_f,
        ]
    )
