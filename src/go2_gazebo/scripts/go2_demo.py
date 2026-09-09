#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
go2_demo.py -- publish a /cmd_vel patrol path for `seconds` seconds.

Lets an unattended run (or the launch `demo:=<seconds>` argument) show the Go2
gliding + trotting: drives straight ahead, then does a gentle curve so the lidar
sweeps the obstacles and the odometry integration visibly moves.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Demo(Node):
    def __init__(self, seconds: float):
        super().__init__('go2_demo')
        self.seconds = seconds
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.t0 = self.get_clock().now()
        self.get_logger().info(
            'Demo: publishing /cmd_vel for %.1f s (forward, then a curve)...'
            % seconds)

    def vel_for(self, t: float) -> Twist:
        msg = Twist()
        if t >= self.seconds:
            msg.linear.x = 0.0
            return msg
        if t < 6.0:
            msg.linear.x = 0.35          # first: straight ahead
        else:
            # slow curve so the robot patrols around the obstacles
            msg.linear.x = 0.25
            msg.angular.z = 0.3 * math.sin(2.0 * math.pi * (t - 6.0) / 24.0)
        return msg

    def step(self) -> bool:
        """Publish one sample; return False when the demo is over."""
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if t >= self.seconds:
            self.pub.publish(Twist())
            self.get_logger().info('Demo finished (velocity zeroed).')
            return False
        self.pub.publish(self.vel_for(t))
        return True


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    if seconds <= 0.0:
        return  # demo disabled (launch default demo:=0)
    rclpy.init()
    node = Demo(seconds)
    try:
        while rclpy.ok() and node.step():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
