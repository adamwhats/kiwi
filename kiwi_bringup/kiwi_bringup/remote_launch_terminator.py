#!/usr/bin/env python3

# Based on afrixs's answer from https://robotics.stackexchange.com/questions/97405/remotely-launch-nodes-in-ros2

import rclpy
from rclpy.node import Node
import os


def main(args=None):
    rclpy.init(args=args)

    node = Node('remote_launch_terminator')
    node.declare_parameter('screen_pid', 'remote')
    screen_pid = node.get_parameter('screen_pid').get_parameter_value().string_value

    try:
        rclpy.spin(node)
    except:
        pass

    print('Terminating ' + screen_pid)
    os.system('screen -S ' + screen_pid + ' -X quit')
    os.system('bash -i -c "ssh -t dev@10.42.0.54 \'bash -i -c \\"cd ~/kiwi_ws/ && docker compose -f src/kiwi/kiwi_dockerfiles/docker-compose.yaml down \\"\'"')


if __name__ == '__main__':
    main()
