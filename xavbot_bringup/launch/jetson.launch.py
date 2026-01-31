import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    # Realsense
    realsense_node = Node(
        name='realsense',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'camera_name': 'camera',
            'base_frame_id': 'link',
            'serial_no': '_008222072206',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_infra1': False,
            'enable_infra2': False,
            'pointcloud.enable': True,
            '_image_transport': 'compressed',
            'publish_tf': True,
        }]
    )

    # Lidar
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('rplidar_ros2_driver'), 'launch', 'rplidar.launch.py'])),
        launch_arguments={
            'params_file': PathJoinSubstitution([FindPackageShare('xavbot_bringup'), 'config', 'lidar.yaml'])
        }.items(),
    )

    lidar_with_ns = GroupAction([PushRosNamespace('rplidar'), lidar_node])

    # lidar_odom = Node(
    #     package='rf2o_laser_odometry',
    #     executable='rf2o_laser_odometry_node',
    #     name='rf2o_laser_odometry',
    #     output='screen',
    #     parameters=[{
    #         'laser_scan_topic': '/scan',
    #         'odom_topic': '/rf2o_odom',
    #         'publish_tf': False,
    #         'base_frame_id' : 'base_link',
    #         'init_pose_from_topic' : '',
    #         'freq' : 8.0
    #     }],
    #     arguments=['--ros-args', '--log-level', 'error']
    # )

    # # EKF
    # ekf = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[PathJoinSubstitution([FindPackageShare('xavbot_bringup'), 'config', 'ekf.yaml'])],
    # )

    # Navigation
    use_nav2 = LaunchConfiguration('navigation', default=False)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'params_files': PathJoinSubstitution([FindPackageShare('xavbot_bringup'), 'config', 'nav2_params.yaml']),
            'map': 'False'
        }.items(),
        condition=IfCondition(use_nav2)
    )

    # Foxglove bridge - disable 'services' capability to avoid typesupport errors
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])
        ]),
        launch_arguments={
            'capabilities': '[clientPublish,parameters,parametersSubscribe,connectionGraph,assets]',  # Removed services
        }.items()
    )

    return LaunchDescription([
        realsense_node,
        lidar_with_ns,
        foxglove_bridge,
    ])
