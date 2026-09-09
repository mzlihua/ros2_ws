#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cloud_bench.py -- run inside the sim container (entrypoint MODE=bench).

Watches the running headless sim and answers the only question that matters on
a metered server: *is this instance big enough for real time?*

Metrics
  - real-time factor (RTF): sim-clock advance vs wall-clock, read from the gz
    /clock bridge.  Sensor header stamps are NOT usable here (verified: this
    gz-sim 10 stack emits non-monotonic /scan_raw & /imu stamps), so /clock is
    the authoritative sim time source.
  - /scan_raw & /imu/data wall arrival rates + lidar richness (finite rays).
  - free memory + load average (billing/memory guardrail).
  - drive sanity: publish /cmd_vel at 0.5 m/s for 4 s, confirm /odom integrates
    (go2_driver integrates on a wall-clock timer, so ~2 m is expected).

Verdict PASS/FAIL; exit code 0 = this instance can host the sim, 1 otherwise.
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
from rosgraph_msgs.msg import Clock

BE = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
DRIVE_SPEED = 0.5   # m/s
DRIVE_SECS = 4.0


class Bench(Node):
    def __init__(self):
        super().__init__('cloud_bench')
        self.scan_frames = 0
        self.imu_count = 0
        self.odom = None
        self.clk = []          # sim-time samples from /clock
        self.latest_scan = None
        self.create_subscription(LaserScan, '/scan_raw', self._scan, BE)
        self.create_subscription(Imu, '/imu/data', self._imu, BE)
        self.create_subscription(Odometry, '/odom', self._odom, BE)
        self.create_subscription(Clock, '/clock', self._clock_cb, BE)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def _scan(self, m):
        self.scan_frames += 1
        self.latest_scan = m

    def _imu(self, _m):
        self.imu_count += 1

    def _odom(self, m):
        self.odom = m

    def _clock_cb(self, m):   # NB: do NOT name this _clock (rclpy uses it)
        self.clk.append(m.clock.sec + m.clock.nanosec * 1e-9)
        del self.clk[:-4000]


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
        if node.scan_frames >= 3 and node.imu_count > 0 and \
                node.odom is not None and len(node.clk) >= 20:
            return True
    return False


def main():
    rclpy.init()
    b = Bench()
    if not wait_ready(b, timeout=60):
        try:
            with open('/tmp/sim.log') as fh:
                tail = ''.join(fh.readlines()[-15:])
        except OSError:
            tail = '(no /tmp/sim.log)'
        print('SIM NEVER CAME UP. last sim log:\n%s' % tail)
        return 1

    # ---- warm up ~8 s (shader compile / catch-up inflate early RTF) --------
    t_warm = time.monotonic()
    while time.monotonic() - t_warm < 8.0:
        rclpy.spin_once(b, timeout_sec=0.1)

    # ---- measure window: rates + RTF over ~6 wall seconds ------------------
    b.scan_frames = 0
    b.imu_count = 0
    b.clk = []
    w0 = time.monotonic()
    while time.monotonic() - w0 < 6.0:
        rclpy.spin_once(b, timeout_sec=0.05)
    wall = time.monotonic() - w0
    scan_hz = b.scan_frames / wall
    imu_hz = b.imu_count / wall
    if len(b.clk) >= 2:
        sim = b.clk[-1] - b.clk[0]
        rtf = sim / wall if wall > 0 else 0.0
    else:
        sim, rtf = 0.0, 0.0
    clock_ok = len(b.clk) >= 2

    # lidar richness: finite rays on the fullest frame we saw
    fin = -1
    if b.latest_scan is not None:
        fin = sum(math.isfinite(r) for r in b.latest_scan.ranges)
        total = len(b.latest_scan.ranges)
    else:
        total = -1

    # ---- drive sanity: continuous 0.5 m/s for 4 s, watch /odom -------------
    x0 = b.odom.pose.pose.position.x if b.odom else 0.0
    c = Twist(); c.linear.x = DRIVE_SPEED
    sent = 0
    d0 = time.monotonic()
    while time.monotonic() - d0 < DRIVE_SECS:
        b.cmd.publish(c)
        sent += 1
        rclpy.spin_once(b, timeout_sec=0.02)
    x1 = b.odom.pose.pose.position.x if b.odom else x0
    stop = Twist()
    for _ in range(5):
        b.cmd.publish(stop)
        rclpy.spin_once(b, timeout_sec=0.02)
    travelled = x1 - x0
    drive_ok = travelled > 0.8   # ~2.0 m expected at 0.5 m/s over 4 s

    # ---- verdicts -----------------------------------------------------------
    issues = []
    if fin < 20:
        issues.append('lidar returns too sparse (%d finite)' % fin)
    if clock_ok:
        if rtf < 0.4:
            rtf_v = 'FAIL'
            issues.append('RTF %.2f too low -> raise vCPU' % rtf)
        elif rtf < 0.85:
            rtf_v = 'WEAK'
            issues.append('RTF %.2f < 0.85 -> raise vCPU, or disable cameras' % rtf)
        else:
            rtf_v = 'OK'
    else:
        rtf_v = 'n/a'
        issues.append('no /clock messages (bridge?) -> RTF unknown')
    if not drive_ok:
        issues.append('odom moved %.2f m in %.0f s at %.1f m/s -> check driver/cmd'
                      % (travelled, DRIVE_SECS, DRIVE_SPEED))

    print('\n========== cloud_bench ==========')
    print('rtf    : %s' % (('%.2f (%s)' % (rtf, rtf_v)) if clock_ok else rtf_v))
    print('lidar  : %.1f Hz (wall) | %d finite rays / %d' % (scan_hz, fin, total))
    print('imu    : %.1f Hz (wall)' % imu_hz)
    print('drive  : %d cmds over %.0f s -> odom x %.2f m (%s)'
          % (sent, DRIVE_SECS, travelled,
             'OK' if drive_ok else 'check driver'))
    print('mem    : %d MB free | load %s' % (mem_free_mb(), ' '.join(loadavg())))
    for i in issues:
        print('  -', i)
    verdict = 'PASS' if not issues else 'FAIL'
    print('==================================')
    print('VERDICT:', verdict)
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
