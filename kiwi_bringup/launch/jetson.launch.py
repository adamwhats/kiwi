from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    # Lidar
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('rplidar_ros2_driver'), 'launch', 'rplidar.launch.py'])),
        launch_arguments={
            'params_file': PathJoinSubstitution([FindPackageShare('kiwi_bringup'), 'config', 'lidar.yaml'])
        }.items(),
    )

    lidar_with_ns = GroupAction([PushRosNamespace('rplidar'), lidar_node])

    # Slam
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('kiwi_bringup'), 'launch', 'slam.launch.py'])),
    )

    # Perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('kiwi_perception'), 'launch', 'perception.launch.py'])),
    )

    # Navigation
    use_nav2 = LaunchConfiguration('navigation', default=False)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'params_files': PathJoinSubstitution([FindPackageShare('kiwi_bringup'), 'config', 'nav2_params.yaml']),
            'map': 'False'
        }.items(),
        condition=IfCondition(use_nav2)
    )

    # Foxglove bridge
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])
        ]),
        launch_arguments={
            'capabilities': '[clientPublish,parameters,parametersSubscribe,connectionGraph,assets]',  # Removed services
        }.items()
    )

    return LaunchDescription([
        lidar_with_ns,
        slam_launch,
        perception_launch,
        foxglove_bridge,
    ])
