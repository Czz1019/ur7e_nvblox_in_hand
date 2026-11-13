#!/usr/bin/env python3
"""Measure actual ESDF reception using RELIABLE raw subscriptions and wall time."""
import argparse
from collections import deque
import math
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0., help='Seconds; 0 runs until Ctrl+C')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    rclpy.init(args=[])
    node = rclpy.create_node('ur7e_esdf_reliable_hz')
    arrivals = deque()
    state = {'count': 0, 'bytes': 0, 'last': None}

    def received(raw):
        now = time.monotonic()
        arrivals.append(now)
        state.update(count=state['count'] + 1, bytes=len(raw), last=now)

    subscription = node.create_subscription(
        PointCloud2, '/ur7e_nvblox/esdf_points', received,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE), raw=True)
    began = time.monotonic()
    next_report = began + 1.
    previous_count = 0
    previous_report = began
    print('Actual receiver Hz; RELIABLE, raw=True, wall time. Adds one map subscriber.', flush=True)
    try:
        while rclpy.ok() and (args.duration == 0 or time.monotonic() - began < args.duration):
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            while arrivals and arrivals[0] < now - 5.:
                arrivals.popleft()
            if now >= next_report:
                hz = (state['count'] - previous_count) / (now - previous_report)
                window_hz = ((len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
                             if len(arrivals) > 1 and arrivals[-1] > arrivals[0] else 0.)
                age = 'none' if state['last'] is None else f'{now-state["last"]:.2f}s'
                print(f'{now-began:5.1f}s | interval={hz:6.2f}Hz | '
                      f'last5s={window_hz:6.2f}Hz | received={state["count"]} | '
                      f'payload={state["bytes"]/1e6:.2f}MB | last_age={age}', flush=True)
                if hz == 0:
                    print('No new cloud in this interval; check live metrics, /clock and matching environment.', flush=True)
                previous_count, previous_report = state['count'], now
                next_report = now + 1.
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
