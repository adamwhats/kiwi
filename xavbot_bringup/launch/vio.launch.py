# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    # realsense_camera_node = Node(
    #     name='VIO_camera',
    #     namespace='VIO_camera',
    #     package='realsense2_camera',
    #     executable='realsense2_camera_node',
    #     parameters=[{
    #             'enable_infra1': True,
    #             'enable_infra2': True,
    #             'enable_color': False,
    #             'enable_depth': False,
    #             'depth_module.emitter_enabled': 0,
    #             'depth_module.profile': '640x360x90',
    #             'enable_gyro': True,
    #             'enable_accel': True,
    #             'gyro_fps': 200,
    #             'accel_fps': 63,
    #             'unite_imu_method': 2,
    #             'serial_no': '_008222072206',
    #             'camera_name': 'VIO_camera',
    #             '_image_transport': 'compressed',
    #     }]
    # )

    # NOTE: Requires realsense-ros branch >= 4.54.1 to enable static_tf and intra_process_comms
    # This does
    realsense_camera_node = ComposableNode(
        name='VIO_camera',
        namespace='VIO_camera',
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'infra1_qos': 'SENSOR_DATA',
                'infra2_qos': 'SENSOR_DATA',
                'enable_color': False,
                'enable_depth': False,
                'depth_module.emitter_enabled': 0,
                'depth_module.profile': '640x360x90',
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 63,
                'unite_imu_method': 2,
                'serial_no': '_008222072206',
                'camera_name': 'VIO_camera',
        }]
    )

    vio_camera_encoder_node = ComposableNode(
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        name='vio_camera_encoder_node',
        parameters=[{
            'input_width': 640,
            'input_height': 360,
        }],
        remappings=[
            ('image_raw', 'VIO_camera/infra1/image_rect_raw'),
            ('image_compressed', 'VIO_camera/infra1/image_compressed')]
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        extra_arguments=[{"use_intra_process_comms": True}],
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'force_planar_mode': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'input_imu_frame': 'VIO_camera_gyro_optical_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.0003241883170058698,
                    'gyro_random_walk': 8.793450574736146e-06,
                    'accel_noise_density': 0.002997380411615089,
                    'accel_random_walk': 0.00029396920175504343,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 22.00
                    }],
        remappings=[('stereo_camera/left/image', 'VIO_camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'VIO_camera/infra1/camera_info'),
                    ('stereo_camera/right/image', 'VIO_camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'VIO_camera/infra2/camera_info'),
                    ('visual_slam/imu', 'VIO_camera/imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            realsense_camera_node,
            # vio_camera_encoder_node,
            # visual_slam_node
        ],
        output='screen'
    )

    publish_dummy_tf = LaunchConfiguration('standalone', default=False)
    dummy_tf = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'VIO_camera_link'],
                    condition=IfCondition(publish_dummy_tf)
                    )

    return launch.LaunchDescription([
        visual_slam_launch_container,
        # realsense_camera_node,
        # dummy_tf
    ])
