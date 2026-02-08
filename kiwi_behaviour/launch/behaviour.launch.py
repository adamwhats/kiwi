import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tree_xml = os.path.join(get_package_share_directory('kiwi_behaviour'), 'trees', 'nav_to_pose_with_arm.xml')

    arm_move_named_server = Node(
        package='kiwi_behaviour',
        executable='arm_move_named_server',
        output='screen',
    )

    task_executor = Node(
        package='kiwi_behaviour',
        executable='task_executor',
        output='screen',
        parameters=[
            {
                'tree_xml_file': tree_xml,
                'server_timeout_ms': 20000,
                'bt_loop_duration_ms': 10,
                'wait_for_service_timeout_ms': 1000,
            }
        ],
    )

    return LaunchDescription(
        [
            arm_move_named_server,
            task_executor,
        ]
    )
