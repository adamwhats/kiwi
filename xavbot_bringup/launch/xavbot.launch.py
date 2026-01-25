import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    # Visual-inertial odometry
    use_vio = LaunchConfiguration('odom', default=False)
    vio_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('xavbot_bringup'), 'launch', 'vio.launch.py'])
        ),
        condition=IfCondition(use_vio)
    )
    # dummy_odom_tf = Node(package='tf2_ros',
    #                     executable='static_transform_publisher',
    #                     arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
    #                     condition=UnlessCondition(use_vio))

    # Lidar
    lidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'])),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar'
        }.items(),
    )
    lidar_odom = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/rf2o_odom',
            'publish_tf': False,
            'base_frame_id' : 'base_link',
            'init_pose_from_topic' : '',
            'freq' : 8.0
        }],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # EKF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('xavbot_bringup'), 'config', 'ekf.yaml'])],
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

    # Foxglove bridge
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            ])
        ])
    )


    actions = [
        # xavbot_pi_remote_launch,
        # remote_launch_terminator,'
        vio_bringup,
        foxglove_bridge
        # lidar_bringup,
        # lidar_odom,
        # ekf,
        # nav2_bringup,
    ]

    return LaunchDescription(actions)
