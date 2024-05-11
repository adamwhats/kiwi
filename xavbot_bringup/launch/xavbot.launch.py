from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    # Bringup hardware control and Moveit on xavbot raspberry pi
    # Based on afrixs's answer from https://robotics.stackexchange.com/questions/97405/remotely-launch-nodes-in-ros2
    xavbot_pi_remote_launch = ExecuteProcess(
        name='xavbot_pi_remote_launch',
        cmd=['{ outer_stdout=$(readlink -f /proc/self/fd/3); } 3>&1 && screen -DmS xavbot_pi_remote_launch bash -i -c "ssh -t dev@10.42.0.54 \'bash -i -c \\"cd ~/xavbot_ws/ && docker compose -f src/xavbot/xavbot_dockerfiles/docker-compose.yaml up --force-recreate\\"\' > $outer_stdout"'],
        output='screen',
        shell=True,
        emulate_tty=True,
    )
    
    remote_launch_terminator = Node(
        package='xavbot_bringup',
        executable='remote_launch_terminator.py',
        name='remote_launch_terminator',
        output='screen',
        parameters=[{'screen_pid': 'xavbot_pi_remote_launch'}],
        sigterm_timeout='30'
    )

    # Visual-inertial odometry
    use_vio = LaunchConfiguration('odom', default=True)
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('xavbot_bringup'), 'launch', 'vio.launch.py'])
        ),
        condition=IfCondition(use_vio)
    )
    dummy_odom_tf = Node(package='tf2_ros',
                         executable='static_transform_publisher',
                         arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
                         condition=UnlessCondition(use_vio)
                         )
    
    # Navigation
    use_nav2 = LaunchConfiguration('navigation', default=True)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])), launch_arguments={
                'params_files': PathJoinSubstitution([FindPackageShare('xavbot_bringup'), 'config', 'nav2_params.yaml']),
                'map': 'False'
        }.items(),
        condition=IfCondition(use_nav2)
    )

    # Perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('xavbot_perception'), 'launch', 'arm_perception.launch.py']
            )
        )
    )

    actions = [
        xavbot_pi_remote_launch,
        remote_launch_terminator,
        vio_launch,
        dummy_odom_tf,
        # nav2_bringup,
        perception_launch,
    ]

    return LaunchDescription(actions)
