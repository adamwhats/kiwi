from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    use_rviz = LaunchConfiguration('rviz', default=True)

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kiwi_bringup"), "rviz", "config.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    nodes = [
        rviz_node,
    ]

    return LaunchDescription(nodes)
