#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
go2_scan_noise.py -- add gaussian range noise to the gz lidar stream.

gz-sim's GPU lidar does not apply an SDF <noise> element, so the noise is
injected on the ROS side instead: this node subscribes to the raw bridged
scan (from /scan_raw) and republishes a noised copy on /scan (the topic the
rest of the system and RViz consume). /scan_raw stays available as the clean
echo when you need a noiseless reference.

Parameters:
  scan_noise_std  range gaussian stddev in metres (0 = pure pass-through)
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanNoise(Node):
    def __init__(self):
        super().__init__('go2_scan_noise')
        self.declare_parameter('scan_noise_std', 0.015)
        self.std = max(0.0,
                       self.get_parameter('scan_noise_std').value)

        sub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            LaserScan, '/scan_raw', self._cb, sub_qos)
        self.pub = self.create_publisher(LaserScan, '/scan', pub_qos)
        self.get_logger().info('scan noise std = %.4f m' % self.std)

    def _cb(self, msg):
        if self.std <= 0.0:
            self.pub.publish(msg)
            return
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        rng = msg.ranges
        rmax = msg.range_max
        rmin = msg.range_min
        n = len(rng)
        noisy = [0.0] * n
        for i, r in enumerate(rng):
            if math.isfinite(r) and rmin <= r <= rmax:
                noisy[i] = max(rmin, r + random.gauss(0.0, self.std))
            else:
                noisy[i] = r
        out.ranges = noisy
        out.intensities = list(msg.intensities)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
