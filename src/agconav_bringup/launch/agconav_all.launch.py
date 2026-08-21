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
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler, SetEnvironmentVariable, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _no_shm_env():
    """FastDDS 의 공유메모리 전송을 끄고 UDPv4 만 쓰게 한다.

    !! 이게 없으면 지상 로봇 단계에서 파이프라인이 멈춘다 !!
    스테이지 게이트가 풀리는 순간 DDS 참가자가 수십 개(모듈 B/C + Nav2 2세트
    + 지상 로봇 스택) 한꺼번에 생긴다. 그때 FastDDS 가 SHM 포트를 못 열고
        RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_portNNNNN:
        open_and_lock_file failed
    를 낸다. 그 여파로 **이미 떠 있던 /clock 브리지의 발행 엔드포인트까지
    디스커버리에서 빠진다.** 실측: ROS /clock 이 발행자 0 / 구독자 120 인
    상태가 되고, gz 쪽 controller_manager 가 "No clock received" 를 1201회
    찍은 뒤 지상 로봇 주행 단계에 들어가지 못했다. 그때도 gz 프로세스와
    브리지 프로세스 자체는 멀쩡했고 gz 내부 /clock 도 정상 발행 중이었다.

    **속도 손해는 없다. 오히려 빠르다.**
    드론 지도와 같은 규모(112 MB)를 루프백으로 보내 재 보면
        SHM 켜짐  5.20 / 6.11 초
        SHM 꺼짐  3.95 / 3.72 초
    로 UDP 쪽이 35%가량 빨랐다. 루프백에서는 SHM 의 세그먼트 할당·잠금
    오버헤드가 이득보다 크다.

    환경변수가 이미 있으면 존중한다(다른 프로파일을 쓸 수 있다).
    """
    if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        return []
    profile = os.path.join(
        get_package_share_directory("agconav_bringup"),
        "config", "fastdds_no_shm.xml")
    return [SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", profile)]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")
    enable_drone = LaunchConfiguration("enable_drone")
    enable_mapping = LaunchConfiguration("enable_mapping")
    enable_fusion = LaunchConfiguration("enable_fusion")
    enable_traversability = LaunchConfiguration("enable_traversability")
    auto_goals = LaunchConfiguration("auto_goals")
    sequential_pipeline = LaunchConfiguration("sequential_pipeline")
    spawn_ground_early = LaunchConfiguration("spawn_ground_early")

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
        # 근거: docs/10~12 번 문서.
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
        DeclareLaunchArgument("leg_z", default_value="6.2612"),
        DeclareLaunchArgument("leg_yaw", default_value="-0.0767"),
        DeclareLaunchArgument(
            # !! agconav_sim 의 기본값을 여기서 덮어쓴다 !!
            # 이 값이 sim 쪽 기본값보다 우선하므로, 컨트롤러를 바꿀 때
            # 두 파일을 같이 고쳐야 한다. 예전에 sim 만 바꿨더니
            # 전체 실행에서는 그대로 예전 값이 떴다.
            # !! rl 로 유지할 것. guide 로 바꾸지 말 것 !!
            # 등판만 보면 guide 가 낫다 — docs/13 실측으로 guide 20도,
            # RL robot_lab 15도. 그런데 **guide 는 Nav2 주행에서 못 쓴다.**
            # 전체 파이프라인(191 m, 스폰 -> 목표)에서 guide 는 반복해서
            # 전복했고(roll 78 -> 140 -> -53도), 같은 지도·같은 Nav2 설정에서
            # RL 은 목표까지 걸어가 Goal succeeded 했다(오차 0.21 m).
            # 지형은 원인이 아니다 — 경로 전 구간 최대 경사 5.2도, 최대 단차
            # 12 mm 로 guide 한계의 1/4 이하다(정답 heightmap 실측).
            # 자세한 경위와 시도한 대책은 docs/14 (한계점) 참고.
            # 그래서 주행성 임계값도 RL 기준으로 내렸다
            # (traversability_leg.yaml: 20도 -> 15도).
            # guide / champ 는 비교·회귀 확인용으로만 남겨 둔다.
            "leg_controller", default_value="rl",
            description="전체 파이프라인 leg 보행 컨트롤러 (종단 주행이 검증된 rl 기본)"),
        # !! 6.0 이다. 10.0 으로 두지 말 것 !!
        # 문서 10(종단 테스트)이 같은 파이프라인을 두 번 돌려 확정한 값이다.
        #   1차 v10 x 6  : wheel 계획 실패 19건, 18.5 m 후 ABORTED
        #   2차 v6  x 3  : wheel 계획 실패 0건, 191.5 m 주행 SUCCEEDED
        # 차이는 파이프라인이 아니라 드론 스캔 조건 하나였다.
        # 문서 9(람다 재실험)도 같은 결론이다 — 속도는 U자이고 최적은
        # 5.2~6.0 m/s, 6 -> 8 m/s 사이에서 제어가 무너진다
        # (선 이탈 0.053 -> 0.312 m, **고도 0.61 -> 3.00 m**).
        # 고도가 3 m 씩 흔들리면 그대로 높이 잡음이 되어 주행성 지도가
        # 벌집이 된다. 실제로 10.0 으로 전체 맵을 뜬 지도는 wheel 자유
        # 37.5% / 점유 27.7% 였고, v6 x 3 지도는 88.5% / 10.1% 였다.
        # 같은 임계값, 같은 slope_window 인데 이만큼 갈린다.
        #
        # **결정값인데 런치 기본값에 반영이 안 돼 있었다.** 그래서 인자를
        # 안 주고 돌리면 매번 실패 조건으로 스캔했다.
        DeclareLaunchArgument("cruise_speed", default_value="6.0",
                              description="드론 순항 속도 [m/s] (문서 10 확정값)"),
        DeclareLaunchArgument("enable_traversability", default_value="true",
                              description="모듈 F (주행성 분석)"),
        DeclareLaunchArgument(
            "auto_goals", default_value="true",
            description="모듈 F 지도 수신 뒤 wheel·leg 목표 자동 전송"),
        DeclareLaunchArgument(
            "simultaneous_goals", default_value="true",
            description="true: wheel·leg 목표를 동시에 전송, false: wheel→leg 순차 전송"),
        DeclareLaunchArgument(
            "sequential_pipeline", default_value="true",
            description="true: F 완료 후 지상 로봇 spawn, false: drone·wheel·leg를 월드에 함께 올림"),
        DeclareLaunchArgument(
            "spawn_ground_early", default_value="false",
            description="지상 모델은 A부터 함께 두고 B/C만 F 뒤에 시작"),
    ]

    # ── 기반: 월드 + 로봇 3종 + 모듈 B ────────────────────────────────
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "agconav_sim.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_nav2": use_nav2,
            "sequential_pipeline": sequential_pipeline,
            "spawn_ground_early": spawn_ground_early,
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
            "leg_controller": LaunchConfiguration("leg_controller"),
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

    automatic_goals = Node(
        package="agconav_navigation",
        executable="pipeline_goal_sender",
        name="pipeline_goal_sender",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "simultaneous": LaunchConfiguration("simultaneous_goals"),
        }],
        condition=IfCondition(auto_goals),
    )

    stage_gate = Node(
        package="agconav_navigation",
        executable="pipeline_stage_gate",
        # !! agconav_sim 도 같은 실행 파일을 띄운다(그쪽 이름은
        #    simulation_stage_gate). 이름이 겹치면 같은 노드가 두 개가 되어
        #    로그가 두 줄씩 찍히고 어느 쪽이 트리거했는지 알 수 없다.
        #    둘은 하는 일이 달라서 둘 다 필요하다 —
        #      sim  쪽: F 완료 -> 지상 로봇 스택(B/C) 기동
        #      all  쪽: F 완료 -> 후속 모듈(D/E) 기동
        #    이름만 확실히 갈라 둔다. !!
        name="pipeline_stage_gate",
        output="screen",
    )

    def _on_stage_gate_exit(event, context):  # noqa: ARG001
        if event.returncode == 0:
            return [
                LogInfo(msg='[agconav_all] A→F 완료 — D, E와 자동 주행을 시작합니다.'),
                module_d,
                module_e,
                automatic_goals,
            ]
        return [Shutdown(reason='pipeline stage gate failed')]

    start_following_stages = RegisterEventHandler(
        OnProcessExit(target_action=stage_gate, on_exit=_on_stage_gate_exit)
    )

    return LaunchDescription(
        _no_shm_env() + declares + [
            simulation,
            rviz,
            module_a,
            module_f,
            stage_gate,
            start_following_stages,
        ]
    )
