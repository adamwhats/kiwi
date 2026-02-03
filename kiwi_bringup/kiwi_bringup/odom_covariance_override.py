#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomCovarianceOverride(Node):
    def __init__(self):
        super().__init__('odom_covariance_override')

        # Declare parameters
        self.declare_parameter('pose_covariance_x', 0.05)
        self.declare_parameter('pose_covariance_y', 0.05)
        self.declare_parameter('pose_covariance_yaw', 0.03)
        self.declare_parameter('twist_covariance_vx', 0.1)
        self.declare_parameter('twist_covariance_vy', 0.1)
        self.declare_parameter('twist_covariance_vyaw', 0.05)
        self.declare_parameter('input_topic', 'input_odom')
        self.declare_parameter('output_topic', 'output_odom')
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('invert_yaw', False)

        # Get parameter values
        self.pose_cov_x = self.get_parameter('pose_covariance_x').value
        self.pose_cov_y = self.get_parameter('pose_covariance_y').value
        self.pose_cov_yaw = self.get_parameter('pose_covariance_yaw').value
        self.twist_cov_vx = self.get_parameter('twist_covariance_vx').value
        self.twist_cov_vy = self.get_parameter('twist_covariance_vy').value
        self.twist_cov_vyaw = self.get_parameter('twist_covariance_vyaw').value
        self.invert_x = self.get_parameter('invert_x').value
        self.invert_y = self.get_parameter('invert_y').value
        self.invert_yaw = self.get_parameter('invert_yaw').value

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.subscription = self.create_subscription(Odometry, input_topic, self.odom_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Odometry, output_topic, qos_profile_sensor_data)

    def odom_callback(self, msg: Odometry):
        # Build 6x6 pose covariance matrix
        pose_cov = [0.0] * 36
        pose_cov[0] = self.pose_cov_x           # x variance
        pose_cov[7] = self.pose_cov_y           # y variance
        pose_cov[14] = 1e6                      # z variance (large = unused in 2D)
        pose_cov[21] = 1e6                      # roll variance (large = unused in 2D)
        pose_cov[28] = 1e6                      # pitch variance (large = unused in 2D)
        pose_cov[35] = self.pose_cov_yaw        # yaw variance

        # Build 6x6 twist covariance matrix
        twist_cov = [0.0] * 36
        twist_cov[0] = self.twist_cov_vx        # vx variance
        twist_cov[7] = self.twist_cov_vy        # vy variance
        twist_cov[14] = 1e6                     # vz variance (large = unused in 2D)
        twist_cov[21] = 1e6                     # wx variance (large = unused in 2D)
        twist_cov[28] = 1e6                     # wy variance (large = unused in 2D)
        twist_cov[35] = self.twist_cov_vyaw     # wz variance

        # Apply inversions if configured
        out_msg = msg
        if self.invert_x:
            out_msg.pose.pose.position.x = -msg.pose.pose.position.x
            out_msg.twist.twist.linear.x = -msg.twist.twist.linear.x
        if self.invert_y:
            out_msg.pose.pose.position.y = -msg.pose.pose.position.y
            out_msg.twist.twist.linear.y = -msg.twist.twist.linear.y
        if self.invert_yaw:
            out_msg.pose.pose.orientation.z = -msg.pose.pose.orientation.z
            out_msg.twist.twist.angular.z = -msg.twist.twist.angular.z

        # Add covariances
        out_msg.pose.covariance = pose_cov
        out_msg.twist.covariance = twist_cov

        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceOverride()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
