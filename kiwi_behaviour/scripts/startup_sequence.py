#!/usr/bin/env python3
"""Run arm home + gripper open/close on startup, then exit."""

import time
import rclpy
from control_msgs.action import GripperCommand
from kiwi_interfaces.action import ArmMoveNamed
from rclpy.action import ActionClient
from rclpy.node import Node


class StartupSequence(Node):
    def __init__(self):
        super().__init__('startup_sequence')

        self.declare_parameter('arm_action', '/arm_move_named_server/cmd')
        self.declare_parameter('gripper_action', '/gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_open_position', 0.025)
        self.declare_parameter('gripper_close_position', 0.005)
        self.declare_parameter('gripper_settle_time', 0.5)

        arm_topic = self.get_parameter('arm_action').value
        gripper_topic = self.get_parameter('gripper_action').value

        self._arm_client = ActionClient(self, ArmMoveNamed, arm_topic)
        self._gripper_client = ActionClient(self, GripperCommand, gripper_topic)

        self.get_logger().info('Waiting for action servers...')
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info('Action servers available')

    def run(self):
        # Move arm to home
        if not self._send_arm_goal('combined', 'home'):
            return False

        # Open gripper
        open_pos = self.get_parameter('gripper_open_position').value
        if not self._send_gripper_goal(open_pos):
            return False

        # Close gripper
        close_pos = self.get_parameter('gripper_close_position').value
        if not self._send_gripper_goal(close_pos):
            return False

        # Move arm to ready
        if not self._send_arm_goal('combined', 'ready'):
            return False

        # Open gripper
        open_pos = self.get_parameter('gripper_open_position').value
        if not self._send_gripper_goal(open_pos):
            return False

        # Close gripper
        close_pos = self.get_parameter('gripper_close_position').value
        if not self._send_gripper_goal(close_pos):
            return False

        # Move arm to home
        if not self._send_arm_goal('combined', 'home'):
            return False

        self.get_logger().info('Startup sequence complete')
        return True

    def _send_arm_goal(self, group_name: str, target_name: str) -> bool:
        goal = ArmMoveNamed.Goal()
        goal.group_name = group_name
        goal.target_name = target_name
        self.get_logger().info(f"Moving '{group_name}' to '{target_name}'...")

        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f'Arm goal failed: {result.message}')
            return False

        self.get_logger().info(f'Arm: {result.message}')
        return True

    def _send_gripper_goal(self, position: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 0.3
        self.get_logger().info(f'Gripper moving to {position:.3f}m...')

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'Gripper at {result.position:.3f}m (reached_goal={result.reached_goal})')

        settle_time = self.get_parameter('gripper_settle_time').value
        time.sleep(settle_time)
        return True


def main():
    rclpy.init()
    node = StartupSequence()
    node.run()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
