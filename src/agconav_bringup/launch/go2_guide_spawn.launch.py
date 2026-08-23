"""Go2(leg)를 **Unitree 공식 unitree_guide 컨트롤러**로 스폰한다.

운용 기본 컨트롤러다. `go2_rl_spawn.launch.py` 를 **그대로 복사해**
컨트롤러만 바꿨다. 의도적으로 베낀 것이다 — 스폰 순서·이벤트 체인·관절
시딩을 직접 다시 짠 하네스로는 로봇이 기립조차 못 했다.
**RL 판을 고치면 이 파일도 같이 봐야 한다.**

왜 RL 대신 이걸 쓰나 (실측, 단계식 5~30도 경사로, 각 3회):
    unitree_guide   20도까지 통과, 25도에서 전복
    RL robot_lab    15도까지 통과, 20도에서 정지
등판이 한 단계(5도) 높다. 다만 한계에서의 실패 양상은 RL 이 낫다 —
RL 은 자세를 유지한 채 멈추고(이탈 0.13~0.62 m) guide 는 전복한다
(이탈 1.51~3.54 m). 주행성 지도의 경사 상한을 20도로 잡아 애초에
25도 구간에 들어가지 않게 하는 것이 전제다.

운용판과 다른 곳은 두 군데뿐이다.
  1. 컨트롤러: rl_quadruped_controller -> unitree_guide_controller
     (model_folder 파라미터는 없다. 학습 정책이 아니라 MPC/균형 제어다.)
  2. 중계 파라미터 두 개. unitree_guide 는 FSM 과 축 단위가 다르다.
       walk_command 4  : FIXEDSTAND --(4)--> TROTTING
                         (RL 은 3 이다. StateFixedStand.cpp 참고)
       norm_linear 0.4 : StateTrotting.cpp 가
                           v_cmd = invNormalize(ly, -0.4, 0.4)
                         로 ly 를 [-1,1] 조이스틱 축으로 읽는다. m/s 를
                         그대로 넣으면 1.0 이 곧 최대치(0.4 m/s)로 잘려
                         두 컨트롤러를 같은 명령으로 비교할 수 없다.
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
    gait_height = LaunchConfiguration('gait_height').perform(context)
    max_vel_x = LaunchConfiguration('max_vel_x').perform(context)
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
    # !! 발 들어올림 높이를 지형에 맞게 올린다 !!
    # 컨트롤러 기본값은 0.08 m 다. 그러면 100 mm 턱에 발끝이 걸려 그대로
    # 넘어진다(실측: 100 mm 단차 3/3 전복, 150 mm 2/2 전복). 발이 턱보다
    # 확실히 높이 올라가야 넘는다.
    #
    # 파라미터를 dict 로 넘기지 않고 YAML 파일로 쓰는 이유: launch_ros 가
    # 리스트를 파이썬 tuple 로 직렬화해서 spawner 가 yaml 파싱 오류로 죽은
    # 적이 있다(RL 판 down_pos 에서 당했다). 파일로 주면 그 경로를 안 탄다.
    ctrl_params = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'guide_ctrl_params.yaml')
    with open(ctrl_params, 'w') as f:
        f.write('unitree_guide_controller:\n  ros__parameters:\n')
        f.write('    gait_height: %s\n' % gait_height)
        f.write('    max_vel_x: %s\n' % max_vel_x)
    controllers = spawner(['unitree_guide_controller'], [ctrl_params])

    relay = Node(package='agconav_navigation',
                 executable='cmd_vel_to_control_input',
                 name='cmd_vel_to_control_input', output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'cmd_vel_topic': '/%s/cmd_vel' % ns,
                              'output_topic': '/control_input',
                              # !! guide 의 설계 최대는 0.4 m/s 다 !!
                              # StateTrotting 이 ly 를 [-1,1] 조이스틱 축으로
                              # 읽으므로(invNormalize(ly, -0.4, 0.4)), 그보다
                              # 큰 m/s 를 주면 축 범위를 넘어 로봇이 앞으로
                              # 기울기만 하고 발을 못 뗀다(실측 진출 0.00 m).
                              # 호출자가 RL 기준 속도(1.0)를 그대로 넘겨도
                              # 안전하도록 여기서 자른다.
                              'max_linear': min(float(max_lin), 0.4),
                              'max_angular': 1.0,
                              # unitree_guide 전용. 위 머리말 참고.
                              'walk_command': 4,
                              'norm_linear': 0.4,
                              # guide 의 yaw 한계(StateTrotting w_yaw_limit).
                              'norm_angular': 0.5}])

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
        # unitree_guide 는 학습 정책을 쓰지 않아 model_folder 가 없다.
        DeclareLaunchArgument('max_linear', default_value='1.0'),
        # 발 들어올림 높이 [m]. 실측으로 정한 값이다.
        #   0.08 (컨트롤러 기본) : 100 mm 턱에 발이 걸려 3/3 전복
        #   0.15                 : 100 mm 통과, 150 mm 전복
        #   0.20 (확정)          : 150 mm 통과, 200 mm 전복, 등판 20도 유지
        #   0.25                 : 오히려 전복 — 너무 높이 들면 불안정해진다
        DeclareLaunchArgument('gait_height', default_value='0.20'),
        # 조이스틱 축 1.0 이 몇 m/s 인가(컨트롤러 기본 0.4).
        DeclareLaunchArgument('max_vel_x', default_value='0.4'),
        OpaqueFunction(function=_setup),
    ])
