#!/usr/bin/env python3
"""Move this Gazebo arm to the checked raised view, then translate laterally along world Y."""
import argparse
import math
import os
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Same observation pose as test/run_gazebo_validation.py, radians.
from pathlib import Path
import json
import numpy as np
PLAN=json.loads((Path(__file__).resolve().parent/'motion_plan.json').read_text())
VIEW=PLAN['center_joints']
YS=np.array(PLAN['y_offsets_m']);QS=np.array(PLAN['joint_samples'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20., help='Sweep duration in wall seconds; 0 only approaches the view')
    parser.add_argument('--amplitude', type=float, default=.08, help='Lateral amplitude in metres, maximum 0.08 m along world Y')
    parser.add_argument('--check-only', action='store_true', help='Check simulation and joints without publishing motion')
    parser.add_argument('--domain-id', type=int, default=135, help='Expected ROS_DOMAIN_ID of the running Gazebo scene (default: 135)')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != str(args.domain_id):
        raise RuntimeError(f'Use the running Gazebo scene domain: ROS_DOMAIN_ID={args.domain_id}')
    if not math.isfinite(args.seconds) or not 0 <= args.seconds <= 300:
        raise ValueError('seconds must be between 0 and 300')
    if not math.isfinite(args.amplitude) or not 0 <= args.amplitude <= PLAN['amplitude_m']:
        raise ValueError('amplitude must be within the checked plan amplitude (metres)')
    rclpy.init(args=[])
    node = rclpy.create_node('ur7e_gazebo_view_sweep')
    state = {'joints': None, 'joint_wall': 0., 'clock': None,
             'clock_wall': 0., 'clock_ticks': 0, 'description': ''}

    def joints(msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            state['joints'] = [values[name] for name in JOINTS]
            state['joint_wall'] = time.monotonic()

    def clock(msg):
        stamp = msg.clock.sec * 10**9 + msg.clock.nanosec
        if state['clock'] is not None and stamp > state['clock']:
            state['clock_ticks'] += 1
            state['clock_wall'] = time.monotonic()
        state['clock'] = stamp

    best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(JointState, '/joint_states', joints, best)
    node.create_subscription(Clock, '/clock', clock, best)
    node.create_subscription(String, '/robot_description',
                             lambda m: state.update(description=m.data), retained)
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
    publisher = None

    def spin_for(duration):
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.02, max(0., end - time.monotonic())))
        if not rclpy.ok():
            raise KeyboardInterrupt

    def fresh():
        return (state['joints'] is not None and state['clock_ticks'] >= 2 and
                time.monotonic() - state['joint_wall'] < 1. and
                time.monotonic() - state['clock_wall'] < 1.)

    def send(values):
        if not fresh():
            raise RuntimeError('Gazebo clock/joint feedback stopped; no further motion commands sent')
        publisher.publish(Float64MultiArray(data=values))

    def approach(target):
        start = list(state['joints'])
        # Smoothstep maximum slope is 1.5. Command speed <= 0.20 rad/s.
        duration = max(3., 1.5 * max(abs(a-b) for a, b in zip(start, target)) / .20)
        began = time.monotonic()
        while time.monotonic() - began < duration:
            u = min(1., (time.monotonic() - began) / duration)
            w = u*u*(3.-2.*u)
            send([a + w*(b-a) for a, b in zip(start, target)])
            spin_for(1./30.)
        send(list(target))
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline:
            spin_for(.1)
            if not fresh():
                raise RuntimeError('Simulation feedback stopped during approach')
            if max(abs(a-b) for a, b in zip(state['joints'], target)) < .03:
                return
        raise RuntimeError('Arm did not reach the observation pose; check active controllers')

    try:
        deadline = time.monotonic() + 20.
        while time.monotonic() < deadline:
            spin_for(.1)
            if fresh() and 'gazebo_ros2_control' in state['description']:
                break
        if not fresh() or 'gazebo_ros2_control' not in state['description']:
            raise RuntimeError('Need this project\'s running Gazebo scene, advancing /clock, robot_description and /joint_states')
        if not controllers.wait_for_service(timeout_sec=3.):
            raise RuntimeError('Controller manager unavailable')
        future = controllers.call_async(ListControllers.Request())
        deadline = time.monotonic() + 5.
        while not future.done() and time.monotonic() < deadline:
            spin_for(.05)
        if not future.done() or future.result() is None:
            raise RuntimeError('Controller query timed out or failed')
        if not any(c.name == 'forward_position_controller' and c.state == 'active'
                   and c.type == 'position_controllers/JointGroupPositionController'
                   for c in future.result().controller):
            raise RuntimeError('Expected Gazebo forward_position_controller is not active')
        node.get_logger().info(f'Simulation ready. Current joints: {state["joints"]}; view: {VIEW}')
        if args.check_only:
            return
        publisher = node.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 1)
        deadline = time.monotonic() + 5.
        while not publisher.get_subscription_count() and time.monotonic() < deadline:
            spin_for(.1)
        if not publisher.get_subscription_count():
            raise RuntimeError('No position controller command subscriber')
        node.get_logger().info('Approaching the previous observation pose smoothly...')
        if max(abs(a-b) for a,b in zip(state['joints'],PLAN['initial_joints']))<.03:
            for point in PLAN['approach_route'][1:]:approach(point)
        elif max(abs(a-b) for a,b in zip(state['joints'],VIEW))<.03:approach(VIEW)
        else:raise RuntimeError('Current joints do not match checked initial or raised centre pose')
        node.get_logger().info(f'Sweeping world Y by +/-{args.amplitude} m at raised constant-height view for {args.seconds} wall seconds')
        began = time.monotonic()
        while time.monotonic() - began < args.seconds:
            elapsed = time.monotonic() - began
            y_offset=args.amplitude * math.sin(elapsed * .2)
            target=[float(np.interp(y_offset,YS,QS[:,i])) for i in range(6)]
            send(target)
            spin_for(1./30.)
        approach(VIEW)
        node.get_logger().info('Finished. Holding the observation pose; map and perception remain running.')
    except KeyboardInterrupt:
        node.get_logger().info('Stopped publishing; position controller retains its last target.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
