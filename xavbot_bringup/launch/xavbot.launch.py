from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    # Visual-inertial odometry
    use_vio = LaunchConfiguration('odom', default=True)
    vio_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('xavbot_bringup'), 'launch', 'vio.launch.py'])
        ),
        condition=IfCondition(use_vio)
    )
    dummy_odom_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
                        condition=UnlessCondition(use_vio))
    

    # Lidar
    lidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'])),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar'
        }.items(),
    )
    
    # Navigation
    use_nav2 = LaunchConfiguration('navigation', default=True)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])), 
        launch_arguments={
            'params_files': PathJoinSubstitution([FindPackageShare('xavbot_bringup'), 'config', 'nav2_params.yaml']),
            'map': 'False'
        }.items(),
        condition=IfCondition(use_nav2)
    )


    actions = [
        # xavbot_pi_remote_launch,
        # remote_launch_terminator,
        vio_bringup,
        dummy_odom_tf,
        lidar_bringup,
        nav2_bringup,
    ]

    return LaunchDescription(actions)
