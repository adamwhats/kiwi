from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    nav2_params = PathJoinSubstitution(
        [FindPackageShare('kiwi_bringup'), 'config', 'nav2_params.yaml']
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
            ]),
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    twist_stamper = Node(
        package='kiwi_bringup',
        executable='twist_stamper.py',
        name='twist_stamper',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/base_controller/reference',
            'frame_id': 'base_link',
        }],
    )

    return LaunchDescription([nav2_launch, twist_stamper])
