#!/usr/bin/env python3
"""Convert Twist to TwistStamped, bridging Nav2 output to ros2_control input."""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', 'cmd_vel_stamped')
        self.declare_parameter('frame_id', 'base_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.sub = self.create_subscription(Twist, input_topic, self.cb, 10)
        self.pub = self.create_publisher(TwistStamped, output_topic, 10)

    def cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
