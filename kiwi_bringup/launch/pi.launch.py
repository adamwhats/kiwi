from typing import List

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def spawn_controllers(controllers: List[str]) -> List[Node]:
    return [Node(package='controller_manager',
                 executable='spawner',
                 arguments=[controller, '--controller-manager', '/controller_manager']
                 ) for controller in controllers]


def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('kiwi_description'), 'urdf', 'kiwi.urdf.xacro']),
        ' arm:=True'
    ]
    )

    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('kiwi_bringup'), 'config', 'kiwi_controller_config.yaml'])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    kiwi_controllers = [
        'base_controller',
        # 'arm_controller',
        # 'gripper_controller'
    ]

    # Delay start of kiwi controllers after `joint_state_broadcaster`
    kiwi_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[*spawn_controllers(kiwi_controllers)],
        )
    )

    moveit_config = MoveItConfigsBuilder('kiwi', package_name='kiwi_moveit_config').to_moveit_configs()
    move_group = generate_move_group_launch(moveit_config)

    temp_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        kiwi_controllers_spawner,
        # move_group,
        temp_tf
    ]

    return LaunchDescription(nodes)
