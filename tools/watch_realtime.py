#!/usr/bin/env python3
"""Show existing Mapper metrics; does not subscribe to maps or command a robot."""
import argparse
import json
import math
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Wall seconds to watch; 0 watches until Ctrl+C')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    rclpy.init(args=[])
    node = rclpy.create_node('ur7e_mapping_realtime_view')
    began = time.monotonic()
    state = {'received': False}
    print('Hz = existing host-call counters; not GPU completion or RViz FPS.', flush=True)
    print(' wall_s | depth_in color_in | depth_fuse color_fuse | esdf_update esdf_pub mesh_pub | exported_points', flush=True)

    def show(message):
        try:
            data = json.loads(message.data)
            hz = data.get('wall_hz', {})
            values = [float(hz.get(key, 0.0)) for key in
                      ('depth_received', 'color_received', 'depth_integrated',
                       'color_integrated', 'esdf_updated',
                       'esdf_points_published', 'mesh_published')]
            points = int(data.get('esdf_exported_points', 0))
            print(f'{time.monotonic()-began:7.1f} | {values[0]:8.2f} {values[1]:8.2f} | '
                  f'{values[2]:10.2f} {values[3]:10.2f} | '
                  f'{values[4]:11.2f} {values[5]:8.2f} {values[6]:8.2f} | {points:,}', flush=True)
            if not state['received']:
                cfg = data.get('configuration', {})
                print('Active config: voxel_size=' + str(cfg.get('voxel_size')) +
                      ', truncation_vox=' + str(cfg.get('truncation_vox')) +
                      ', scene input=' + str(cfg.get('depth_topic')) +
                      ', workspace=' + str(cfg.get('workspace_bounds_type')), flush=True)
            if data.get('error'):
                print('Mapper error: ' + str(data['error']), flush=True)
            state['received'] = True
        except (ValueError, TypeError, KeyError) as error:
            node.get_logger().warning('Cannot parse Mapper metrics: ' + str(error))

    subscription = node.create_subscription(
        String, '/ur7e_nvblox/metrics', show,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    warned = False
    try:
        while rclpy.ok() and (args.duration == 0 or time.monotonic()-began < args.duration):
            rclpy.spin_once(node, timeout_sec=0.2)
            if not state['received'] and not warned and time.monotonic()-began > 5:
                print('No metrics yet: check matching ROS_DOMAIN_ID, Mapper and live /clock. '
                      'With use_sim_time=true, a missing/paused clock also stops the metrics timer.', flush=True)
                warned = True
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
