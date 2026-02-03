from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

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

    rtabmap_node = Node(
        package='rtabmap_slam',
        namespace='rtabmap',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # Frames
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # TF behavior
            'publish_tf': True,
            'subscribe_odom_info': False,

            # Use external odometry from EKF
            'odom_sensor_sync': False,

            # Sensors to use
            'subscribe_rgb': True,
            'subscribe_depth': True,
            'subscribe_scan': True,
            'subscribe_imu': False,

            # Synchronization
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'topic_queue_size': 30,
            'sync_queue_size': 30,

            # Best effort
            'qos': 2,
            'qos_scan': 2,

            # SLAM parameters
            'RGBD/NeighborLinkRefining': 'true',
            'Reg/Force3DoF': 'true',

            # Loop closure tuning
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/OptimizeFromGraphEnd': 'true',
            'RGBD/ProximityBySpace': 'true',

            # Feature matching
            'Vis/MaxFeatures': '1000',
            'Vis/MinInliers': '20',
            'Reg/Strategy': '2',

            # ICP scan matching
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/Iterations': '30',
            'Icp/VoxelSize': '0.05',

            # Grid/OctoMap
            'Grid/3D': 'true',
            'Grid/Sensor': '2',
            'Grid/RayTracing': 'true',
            'Grid/MaxGroundHeight': '0.1',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/RangeMin': '0.1',
            'Grid/RangeMax': '3.0',

            # Saved map path
            'database_path': '',
        }],
        remappings=[
            ('depth/image', '/camera/realsense/aligned_depth_to_color/image_raw'),
            ('odom', '/odometry/filtered'),
            ('rgb/camera_info', '/camera/realsense/color/camera_info'),
            ('rgb/image', '/camera/realsense/color/image_raw'),
            ('scan', '/rplidar/scan')
        ],
    )

    return LaunchDescription([
        rf2o_node,
        rf2o_covariance_override,
        ekf_node,
        rtabmap_node,
    ])
