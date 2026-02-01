from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # OpenVINS node
    openvins_node = Node(
        package='ov_msckf',
        namespace='openvins',
        executable='run_subscribe_msckf',
        output='screen',
        parameters=[{
            'use_stereo': False,
            'max_cameras': 1,
            'save_total_state': False,
            'config_path': PathJoinSubstitution(
                [FindPackageShare('xavbot_bringup'), 'config', 'openvins', 'estimator_config.yaml']),
        }],
        remappings=[
            ('/image', '/camera/realsense/color/image_raw'),
            ('/imu', '/camera/realsense/imu')
        ]
    )

    # OpenVINS hard codes the tf as being between 'global' and 'imu' frames
    # This incorporates it into the tf  tree so the EKF can fuze it with other odometry estimates
    openvins_static_tf = Node(
        package='tf2_ros',
        namespace='openvins',
        executable='static_transform_publisher',
        name='imu_to_camera_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_imu_optical_frame', 'imu']
    )

    # RF2O Laser Odometry node
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        namespace='rf2o',
        executable='rf2o_laser_odometry_node',
        name='laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/rplidar/scan',
            'odom_topic': '/rf2o/odom',
            'publish_tf': False,
            'init_pose_from_topic': '',
            'laser_frame_id': 'lidar',
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'freq': 8.0,
        }],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # Odom covariance override node (rf2o publishes zero covariances)
    rf2o_covariance_override = Node(
        package='xavbot_bringup',
        namespace='rf2o',
        executable='odom_covariance_override.py',
        name='covariance_override',
        output='screen',
        parameters=[{
            'input_topic': '/rf2o/odom',
            'output_topic': '/rf2o/odom_with_covariance',
            'pose_covariance_x': 0.05,
            'pose_covariance_y': 0.05,
            'pose_covariance_yaw': 0.03,
            'twist_covariance_vx': 0.1,
            'twist_covariance_vy': 0.1,
            'twist_covariance_vyaw': 0.05,
            'invert_x': True,
            'invert_y': True,
        }],
    )

    # EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='odom_ekf',
        output='screen',
        parameters=[PathJoinSubstitution(
            [FindPackageShare('xavbot_bringup'), 'config', 'ekf.yaml']
        )],
    )

    return LaunchDescription([
        # openvins_node,
        # openvins_static_tf,
        rf2o_node,
        rf2o_covariance_override,
        ekf_node,
    ])
