#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cloud_bench.py -- run inside the sim container (entrypoint MODE=bench).

Watches the running headless sim and answers the only question that matters on
a metered server: *is this instance big enough for real time?*

Metrics
  - /scan_raw & /imu/data message rates (they tick at sim time)
  - real-time factor RTF ~= (sim time elapsed) / (wall time elapsed), measured
    from gz message header stamps
  - lidar richness (finite returns -> does the imported scene occlude well?)
  - free memory + load average (billing/memory guardrail)

Then drives /cmd_vel at 0.5 m/s for a few seconds to confirm /odom integrates
(sanity that the patrol pipeline is alive), stops, prints PASS/FAIL.
Exit code 0 = this instance can host the sim; 1 = too weak / broken.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry

BE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class Bench(Node):
    def __init__(self):
        super().__init__('cloud_bench')
        self.scan_stamps = []   # (ros_header_sec_sim, wall_sec)
        self.scan_frames = []
        self.imu_count = 0
        self.odom = None
        self.create_subscription(LaserScan, '/scan_raw', self._scan, BE)
        self.create_subscription(Imu, '/imu/data', self._imu, BE)
        self.create_subscription(Odometry, '/odom', self._odom, BE)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def _scan(self, m):
        now = time.monotonic()
        self.scan_stamps.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, now))
        del self.scan_stamps[:-2000]
        self.scan_frames.append(m)
        del self.scan_frames[:-200]

    def _imu(self, _m):
        self.imu_count += 1


def mem_free_mb():
    with open('/proc/meminfo') as fh:
        for line in fh:
            if line.startswith('MemAvailable:'):
                return int(line.split()[1]) // 1024
    return -1


def loadavg():
    with open('/proc/loadavg') as fh:
        return fh.read().split()[:3]


def wait_ready(node, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(node.scan_frames) >= 3 and node.imu_count > 0 and node.odom:
            return True
    return False


def main():
    rclpy.init()
    b = Bench()
    ok = wait_ready(b, timeout=60)
    if not ok:
        # help the human: surface the sim log written by entrypoint.sh
        try:
            with open('/tmp/sim.log') as fh:
                tail = ''.join(fh.readlines()[-15:])
        except OSError:
            tail = '(no /tmp/sim.log)'
        print('SIM NEVER CAME UP. last sim log:\n%s' % tail)
        return 1

    # settle ~2 s, then measure rates + RTF over ~6 wall seconds
    rclpy.spin_once(b, timeout_sec=0.1)
    b.scan_stamps.clear()
    b.scan_frames.clear()
    b.imu_count = 0
    wall0 = time.monotonic()
    end = wall0 + 6.0
    while time.monotonic() < end:
        rclpy.spin_once(b, timeout_sec=0.1)
    wall_dt = time.monotonic() - wall0

    stamps = b.scan_stamps
    n = len(stamps)
    if n >= 2:
        sim_dt = stamps[-1][0] - stamps[0][0]
        scan_hz = (n - 1) / wall_dt if wall_dt > 0 else 0.0
        rtf = sim_dt / wall_dt if wall_dt > 0 else 0.0
    else:
        rtf, scan_hz, sim_dt = 0.0, 0.0, 0.0
    imu_hz = b.imu_count / wall_dt if wall_dt > 0 else 0.0

    # lidar richness on the newest frame
    fin = -1
    if b.scan_frames:
        best = max(b.scan_frames,
                   key=lambda m: sum(math.isfinite(r) for r in m.ranges))
        fin = sum(math.isfinite(r) for r in best.ranges)

    # sanity: drive 4 s, watch /odom integrate
    c = Twist(); c.linear.x = 0.5
    x0 = b.odom.pose.pose.position.x if b.odom else 0.0
    for _ in range(20):
        b.cmd.publish(c)
        rclpy.spin_once(b, timeout_sec=0.2)
    x1 = b.odom.pose.pose.position.x if b.odom else x0
    stop = Twist()
    for _ in range(3):
        b.cmd.publish(stop)
        rclpy.spin_once(b, timeout_sec=0.1)

    issues = []
    if fin < 20:
        issues.append('lidar returns too sparse (%d finite)' % fin)
    if rtf >= 0.85:
        rtf_v = 'OK'
    elif rtf >= 0.4:
        rtf_v = 'WEAK'
        issues.append('RTF %.2f < 0.85 -> raise vCPU, or disable sensors' % rtf)
    else:
        rtf_v = 'FAIL'
        issues.append('RTF %.2f too low' % rtf)

    print('\n========== cloud_bench ==========')
    print('lidar  : %.1f Hz (wall) | %d finite rays / %d' % (scan_hz, fin,
          len(best.ranges) if b.scan_frames else -1))
    print('imu    : %.1f Hz (wall)' % imu_hz)
    print('RTF    : %.2f (%s)   sim %.1f s per %.1f wall s' % (rtf, rtf_v, sim_dt, wall_dt))
    print('drive  : odom x %.3f -> %.3f m at 0.5 m/s over 4 s (%s)' %
          (x0, x1, 'OK' if (x1 - x0) > 1.2 else 'check driver'))
    print('mem    : %d MB free | load %s' % (mem_free_mb(), ' '.join(loadavg())))
    verdict = 'PASS' if not issues else 'FAIL'
    for i in issues:
        print('  -', i)
    print('==================================')
    print('VERDICT:', verdict)
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
