from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Lidar
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare('rplidar_ros2_driver'), 'launch', 'rplidar.launch.py']
            )
        ),
        launch_arguments={
            'params_file': PathJoinSubstitution([FindPackageShare('kiwi_bringup'), 'config', 'lidar.yaml'])
        }.items(),
    )

    lidar_with_ns = GroupAction([PushRosNamespace('rplidar'), lidar_node])

    # Slam
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('kiwi_bringup'), 'launch', 'slam.launch.py'])
        ),
    )

    # Perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare('kiwi_perception'), 'launch', 'perception.launch.py']
            )
        ),
    )

    # Navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('kiwi_bringup'), 'launch', 'navigation.launch.py'])
        ),
    )

    # Foxglove bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[
            {
                'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 'connectionGraph', 'assets'],
            }
        ],
        ros_arguments=['--log-level', 'warn'],
    )

    return LaunchDescription(
        [
            lidar_with_ns,
            slam_launch,
            perception_launch,
            nav2_launch,
            foxglove_bridge,
        ]
    )
