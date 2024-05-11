from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    camera = ComposableNode(name='arm_camera',
                            namespace='arm_camera',
                            package='realsense2_camera',
                            plugin='realsense2_camera::RealSenseNodeFactory',
                            parameters=[{
                                'camera_name': 'arm_camera',
                                'serial_no': '_927522073083',
                                'rgb_camera.profile': '1920x1080x6',
                                'depth_module.profile': '640x360x6',
                                'enable_color': True,
                                'enable_depth': True,
                                'enable_infra1': False,
                                'enable_infra2': False,
                                'pointcloud.enable': True,
                                'pointcloud.stream_filter': 2,  # Use RS2_STREAM_COLOR for pointcloud texture
                                'pointcloud.qos': 'SENSOR_DATA',
                                'decimation_filter.enable': True,
                                'decimation_filter.filter_magnitude': 2,
                                'align_depth': True,
                                'clip_distance': 0.5, 
                            }],
                            # extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    resize = ComposableNode(name='arm_camera_resize',
                            package='isaac_ros_image_proc',
                            plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                            parameters=[{
                                'output_width': 640,
                                'output_height': 360,
                            }],
                            remappings=[
                                ('/image', '/arm_camera/color/image_raw'),
                                ('/camera_info', '/arm_camera/color/camera_info'),
                                ('/resize/image', '/arm_camera/color/image_resize'),
                            ])

    detector = ComposableNode(name='apriltag_detector',
                              package='isaac_ros_apriltag',
                              plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                              parameters=[{
                                  'size': 0.024
                              }],
                              remappings=[
                                  ('/image', '/arm_camera/color/image_raw'),
                                  ('/camera_info', '/arm_camera/color/camera_info'),
                                  ('/tag_detections', '/apriltag_detections'),
                              ],
                            #   extra_arguments=[{'use_intra_process_comms': True}]
    )

    manager = ComposableNode(package='xavbot_perception',
                             plugin='xavbot_perception::AprilTagManager')

    container = ComposableNodeContainer(name='apriltag_detection',
                                        namespace='',
                                        package='rclcpp_components',
                                        executable='component_container',
                                        composable_node_descriptions=[
                                            camera,
                                            resize,
                                            detector,
                                            manager,
                                        ],
                                        output='both',
                                        )

    return LaunchDescription([container])
