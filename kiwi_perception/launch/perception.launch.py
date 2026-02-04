from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    log_level = LaunchConfiguration('log_level')

    perception_nodes = [
        ComposableNode(
            package='realsense2_camera',
            plugin='realsense2_camera::RealSenseNodeFactory',
            name='realsense',
            parameters=[{
                'camera_name': 'camera',
                'base_frame_id': 'link',
                'serial_no': '_008222072206',
                'depth_module.depth_profile': '640x480x15',
                'rgb_camera.color_profile': '640x480x15',
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_depth': True,
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 63,
                'unite_imu_method': 2,
                'align_depth.enable': True,
                'pointcloud__neon_.enable': True,
                'decimation_filter.enable': True,
                'decimation_filter.filter_magnitude': 2,
                'publish_tf': True,
                '_image_transport': 'compressed',
            }],
        ),
        ComposableNode(
            package='kiwi_perception',
            plugin='kiwi_perception::GraspPlanner',
            name='grasp_planner',
            remappings=[
                ('/pointcloud', '/camera/realsense/depth/color/points'),
            ],
            parameters=[{
                'log_level': 'info'
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    pointcloud_container = ComposableNodeContainer(
        name='perception_container',
        namespace='perception',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=perception_nodes,
        output='both',
    )

    return LaunchDescription([
        pointcloud_container,
    ])
