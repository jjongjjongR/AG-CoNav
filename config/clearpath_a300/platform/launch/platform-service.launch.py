from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_common = FindPackageShare('clearpath_common')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        pkg_clearpath_common, 'launch', 'platform.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    # 원래 clearpath 생성기가 생성 당시의 절대경로를 그대로
                    # 박아 넣는다(/home/<사용자>/.../config/clearpath_a300).
                    # 그러면 다른 PC 에서 없는 경로를 가리켜 platform.launch.py
                    # 가 설정을 못 읽는다. 런타임에 설치된 share 를 찾게 바꿨다.
                    # 다시 생성하면 이 수정이 덮이므로, 그때는 여기를 다시 고쳐야
                    # 한다(agconav_bringup/CMakeLists.txt 주석 참고).
                    PathJoinSubstitution([
                        FindPackageShare('agconav_bringup'),
                        'config', 'clearpath_a300'])
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'true'
                )
                ,
                (
                    'namespace'
                    ,
                    'wheel'
                )
                ,
                (
                    # 모듈 B가 자체 ekf_node를 같은 네임스페이스에 띄우므로
                    # clearpath 플랫폼 EKF를 끈다(같은 이름 노드 2개 충돌).
                    # 대신 wheel/odom -> wheel/base_link TF는 아래 control.yaml의
                    # enable_odom_tf 로 diff_drive_controller가 직접 발행한다.
                    'enable_ekf'
                    ,
                    'false'
                )
                ,
                (
                    'use_manipulation_controllers'
                    ,
                    'true'
                )
                ,
            ]
    )

    # Nodes
    node_cmd_vel_bridge = Node(
        name='cmd_vel_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='wheel',
        output='screen',
        arguments=
            [
                'wheel/cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist'
                ,
                '/model/wheel/robot/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist'
                ,
            ]
        ,
        remappings=
            [
                (
                    'wheel/cmd_vel'
                    ,
                    'cmd_vel'
                )
                ,
                (
                    '/model/wheel/robot/cmd_vel'
                    ,
                    'platform/cmd_vel'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_odom_base_tf_bridge = Node(
        name='odom_base_tf_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='wheel',
        output='screen',
        arguments=
            [
                '/model/wheel/robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/model/wheel/robot/tf'
                    ,
                    'tf'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_platform)
    ld.add_action(node_cmd_vel_bridge)
    ld.add_action(node_odom_base_tf_bridge)
    return ld
