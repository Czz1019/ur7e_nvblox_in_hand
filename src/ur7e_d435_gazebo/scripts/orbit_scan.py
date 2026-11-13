#!/usr/bin/env python3
"""Multi-height, target-facing Gazebo scan; --plan-only never starts a ROS node."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from orbit_geometry import Model, Fixture, Guard, JOINTS, design

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
FIXTURE = PACKAGE/'config/collision_world_gazebo_tall.yml'


def readiness(state, now):
    """Explain each input gate without starting ROS or sending commands."""
    checks = {
        '/joint_states': state['q'] is not None and now-state['q_wall'] < 1.,
        '/clock (advancing)': state['ticks'] >= 2 and now-state['clock_wall'] < 1.,
        '/robot_description (Gazebo URDF)': 'gazebo_ros2_control' in state['description'],
        '/camera/depth/camera_info': state['info'] is not None and now-state['info_wall'] < 3.,
        '/nvblox_node/metrics (native, fresh)': (
            isinstance(state['stats'], dict) and state['stats'].get('backend') == 'official_ros_cpp'
            and isinstance(state['stats'].get('totals'), dict) and now-state['stats_wall'] < 3.),
    }
    return dict(checks=checks, missing=[name for name, ok in checks.items() if not ok])


def verify_fixture(fixture):
    """Read-only Gazebo CLI checks. Refuse a moved/resized reference fixture."""
    expected = [('ur', np.zeros(3), np.eye(3), None)]
    expected += [('sage_clean_'+n, c, r, ('box', h*2)) for n, c, h, r in fixture.boxes]
    expected += [('sage_clean_'+n, c, np.eye(3), ('sphere', radius)) for n, c, radius in fixture.spheres]
    number = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
    for name, position, rotation, shape in expected:
        result = subprocess.run(['gz', 'model', '-m', name, '-p'], capture_output=True, text=True, timeout=10, check=True)
        values = result.stdout.strip().split()
        if len(values) != 6: raise RuntimeError(f'Cannot verify Gazebo model pose: {name}')
        values = np.array(list(map(float, values)))
        if (not np.isfinite(values).all() or np.linalg.norm(values[:3]-position) > .002 or
                np.linalg.norm(Rotation.from_matrix(Rotation.from_euler('xyz', values[3:]).as_matrix().T @ rotation).as_rotvec()) > .002):
            raise RuntimeError(f'Model {name} differs from configured scan fixture')
        if shape is None: continue
        info = subprocess.run(['gz', 'model', '-m', name, '-i'], capture_output=True, text=True, timeout=10, check=True).stdout
        if shape[0] == 'box':
            matches = re.findall(r'size\s*\{([^}]+)\}', info)
            sizes = []
            for item in matches:
                xyz = [re.search(r'\b'+axis+r'\s*:\s*('+number+')', item) for axis in 'xyz']
                if all(xyz): sizes.append([float(x.group(1)) for x in xyz])
            good = any(np.allclose(size, shape[1], atol=.001) for size in sizes)
        else:
            radii = re.findall(r'\bradius\s*:\s*('+number+')', info)
            good = any(abs(float(radius)-shape[1]) < .001 for radius in radii)
        if not good: raise RuntimeError(f'Cannot verify primitive dimensions of {name}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--check-only', action='store_true', help='Check live inputs, controller and Gazebo fixture; never publish joint commands')
    parser.add_argument('--wait-timeout', type=float, default=60., help='Live input wait timeout, wall seconds (1..300)')
    parser.add_argument('--replan', action='store_true', help='Ignore a cached plan; plan from current joints when executing')
    parser.add_argument('--views', type=int, default=6, help='Azimuth sectors at EACH of three heights (6..24)')
    parser.add_argument('--dwell', type=float, default=2.5, help='Minimum stopped capture time per view, wall seconds')
    parser.add_argument('--require-full-orbit', action='store_true', help='Refuse partial reachability rather than executing reachable sectors')
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/orbit/offline_plan.json')
    args = parser.parse_args()
    if args.plan_only and args.check_only:
        parser.error('--plan-only and --check-only are mutually exclusive')
    if not math.isfinite(args.wait_timeout) or not 1. <= args.wait_timeout <= 300.:
        parser.error('--wait-timeout must be 1..300 seconds')
    if not 6 <= args.views <= 24 or not math.isfinite(args.dwell) or not 1. <= args.dwell <= 15.:
        raise ValueError('views must be 6..24 and dwell must be 1..15 seconds')
    fixture = Fixture(FIXTURE)
    if args.plan_only:
        import xacro
        description = xacro.process_file(str(PACKAGE/'urdf/ur7e_d435.urdf.xacro')).toxml()
        initial = yaml.safe_load((PACKAGE/'config/initial_positions.yaml').read_text())
        plan = design(Model(description), fixture, np.array([initial[j] for j in JOINTS]),
                      args.views, lambda s: print(s, flush=True))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(plan, indent=2)+'\n')
        print(f'Offline plan saved: {args.output}; accepted {plan["accepted_sectors"]}/{plan["requested_sectors"]}, azimuth span {plan["azimuth_span_deg"]:.1f} deg. No motion executed.')
        return
    if os.environ.get('ROS_DOMAIN_ID') != '127':
        raise RuntimeError('Only this project Gazebo test domain ROS_DOMAIN_ID=127 is supported')
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState, CameraInfo
    from std_msgs.msg import String, Float64MultiArray
    from geometry_msgs.msg import PoseStamped, Point
    from nav_msgs.msg import Path as CameraPath
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init(args=[]); node = rclpy.create_node('ur7e_gazebo_orbit_scan')
    state = dict(q=None, q_wall=0., clock=None, clock_wall=0., ticks=0, description='', stats=None, stats_wall=0., info=None, info_wall=0.)
    best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def joints(msg):
        q = dict(zip(msg.name, msg.position))
        if all(j in q and math.isfinite(q[j]) for j in JOINTS):
            state.update(q=np.array([q[j] for j in JOINTS]), q_wall=time.monotonic())
    def clock(msg):
        old = state['clock']; stamp = msg.clock.sec*10**9+msg.clock.nanosec
        if old is not None and stamp > old.sec*10**9+old.nanosec:
            state.update(ticks=state['ticks']+1, clock_wall=time.monotonic())
        state['clock'] = msg.clock
    def stats(msg):
        try: data = json.loads(msg.data)
        except (ValueError, TypeError): return
        state.update(stats=data, stats_wall=time.monotonic())
    node.create_subscription(JointState, '/joint_states', joints, best)
    node.create_subscription(Clock, '/clock', clock, best)
    node.create_subscription(String, '/robot_description', lambda m: state.update(description=m.data), retained)
    node.create_subscription(String, '/nvblox_node/metrics', stats, 2)
    node.create_subscription(CameraInfo, '/camera/depth/camera_info', lambda m: state.update(info=m, info_wall=time.monotonic()), best)
    path_pub = node.create_publisher(CameraPath, '/nvblox_in_hand/scan/planned_camera_path', retained)
    markers_pub = node.create_publisher(MarkerArray, '/nvblox_in_hand/scan/view_markers', retained)
    status_pub = node.create_publisher(String, '/nvblox_in_hand/scan/status', retained)
    command = None; guard = None
    run_report = dict(started_wall_time=time.time(), captures=[], result='not_executed',
                      motion_commands_sent=0, mode='check_only' if args.check_only else 'scan',
                      environment={key: os.environ.get(key, '') for key in (
                          'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
                          'CYCLONEDDS_URI', 'GAZEBO_MASTER_URI')})
    def spin(seconds):
        deadline = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.02, max(0., deadline-time.monotonic())))
        if not rclpy.ok(): raise KeyboardInterrupt
    def fresh():
        return state['q'] is not None and state['ticks'] >= 2 and time.monotonic()-state['q_wall'] < 1. and time.monotonic()-state['clock_wall'] < 1.
    def status(phase, **extra):
        msg = String(); msg.data = json.dumps(dict(phase=phase, **extra)); status_pub.publish(msg)
        node.get_logger().info(msg.data)
    last_guard_time = 0.
    def send(q):
        nonlocal last_guard_time
        if not fresh(): raise RuntimeError('Simulation feedback stopped; no further commands sent')
        if time.monotonic()-state['stats_wall'] > 3.: raise RuntimeError('Perception statistics stopped; scan aborted')
        if time.monotonic()-last_guard_time > .2:
            if not guard.valid(state['q']): raise RuntimeError('Actual joint feedback fails fixture collision guard')
            last_guard_time = time.monotonic()
        command.publish(Float64MultiArray(data=np.asarray(q).tolist()))
        run_report['motion_commands_sent'] += 1
    def move(target):
        start = state['q'].copy(); target = np.array(target)
        if not guard.edge(start, target, time.monotonic()+15.):
            raise RuntimeError('Actual settled joints do not have a checked straight connection to the next waypoint')
        spin(.1)
        if max(abs(state['q']-start)) > .005:
            raise RuntimeError('Joints changed during segment validation; no new segment command sent')
        duration = max(.8, 1.5*max(abs(target-start))/.12)
        began = time.monotonic()
        while time.monotonic()-began < duration:
            u = min(1., (time.monotonic()-began)/duration); w=u*u*(3.-2.*u)
            send(start+w*(target-start)); spin(1./30.)
        send(target)
        deadline = time.monotonic()+8.
        while time.monotonic() < deadline:
            spin(.1)
            if not fresh(): raise RuntimeError('Feedback stopped waiting for the commanded view')
            if max(abs(state['q']-target)) < .015: return
        raise RuntimeError('Joint tracking did not settle')
    try:
        status('waiting_inputs', environment=run_report['environment'], timeout_seconds=args.wait_timeout)
        deadline = time.monotonic()+args.wait_timeout
        last_notice = 0.
        while time.monotonic() < deadline:
            spin(.1)
            report = readiness(state, time.monotonic())
            run_report['readiness'] = report
            if not report['missing']: break
            if time.monotonic()-last_notice >= 5.:
                status('waiting_inputs', missing=report['missing'])
                last_notice = time.monotonic()
        else:
            raise RuntimeError('Missing or stale inputs: '+', '.join(run_report['readiness']['missing'])+
                '. Start ./run.sh in another terminal with the same ROS_DOMAIN_ID=127; '
                'the scan command does not start Gazebo or the mapper. See docs/OPERATIONS.md')
        info = state['info']
        if (info.header.frame_id != 'camera_depth_optical_frame' or info.k[0] <= 0 or info.k[4] <= 0 or
                info.width/(2*info.k[0]) < .93 or info.height/(2*info.k[4]) < .53):
            raise RuntimeError('Live camera frame/FOV does not match this scan design')
        controller = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not controller.wait_for_service(timeout_sec=3.): raise RuntimeError('Controller manager missing')
        future = controller.call_async(ListControllers.Request()); deadline=time.monotonic()+5.
        while not future.done() and time.monotonic()<deadline: spin(.05)
        if not future.done() or not any(c.name=='forward_position_controller' and c.state=='active' and c.type=='position_controllers/JointGroupPositionController' for c in future.result().controller):
            raise RuntimeError('Expected project simulation controller is not active')
        status('checking_fixture'); verify_fixture(fixture); spin(.3)
        if args.check_only:
            run_report['readiness'] = readiness(state, time.monotonic())
            if run_report['readiness']['missing']:
                raise RuntimeError('Inputs stopped during fixture checks: '+', '.join(run_report['readiness']['missing']))
            run_report.update(result='ready', controller_verified=True, fixture_verified=True)
            status('ready', motion_commands_sent=0, note='Preflight only; route and reconstruction quality not tested')
            return
        model = Model(state['description']); guard = Guard(model, fixture)
        cached = json.loads(args.output.read_text()) if args.output.exists() else None
        if (cached is None or args.replan or cached.get('planner_revision') != 4 or cached.get('height_levels_m') != [.35, .6, .8] or
                cached.get('requested_sectors') != 3*args.views):
            status('planning')
            plan = design(model, fixture, state['q'].copy(), args.views, lambda s: node.get_logger().info(s))
            args.output.parent.mkdir(parents=True, exist_ok=True); args.output.write_text(json.dumps(plan, indent=2)+'\n')
        else: plan = cached
        if plan['joint_names'] != JOINTS or plan.get('schema') != 1: raise RuntimeError('Unsupported scan plan')
        if args.require_full_orbit and not plan['all_requested_sectors_reachable']:
            raise RuntimeError('Some sectors are unreachable; no motion sent (--require-full-orbit)')
        if not np.allclose(plan['target_bounds'], [fixture.low, fixture.high], atol=1e-8):
            raise RuntimeError('Cached plan belongs to another obstacle fixture')
        route = np.array(plan['route'], dtype=float)
        if route.ndim != 2 or route.shape[1] != 6 or not np.isfinite(route).all(): raise ValueError('Invalid route')
        for capture in plan['captures']:
            if not np.allclose(model.camera_pose(route[capture['route_index']]), capture['camera_pose'], atol=1e-6):
                raise RuntimeError('Live URDF/hand-eye differs from the cached plan; run --replan')
        status('validating_route', accepted=plan['accepted_sectors'], requested=plan['requested_sectors'])
        for a, b in zip(route, route[1:]):
            spin(.01)
            if not guard.valid(a) or not guard.valid(b) or not guard.edge(a, b): raise RuntimeError('Cached route fails current geometry/limits')
        spin(.4); start = state['q'].copy()
        connector = guard.connect(start, route[0], np.random.default_rng(128))
        if connector is None: raise RuntimeError('No checked connection from the current joints to the scan route')
        spin(.3)
        if max(abs(state['q']-start)) > .02: raise RuntimeError('Arm moved while planning; stop other command sources and retry')
        if node.count_publishers('/forward_position_controller/commands'): raise RuntimeError('Another joint command publisher is active')
        command = node.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 1)
        spin(1.)
        if command.get_subscription_count() == 0 or node.count_publishers('/forward_position_controller/commands') > 1:
            raise RuntimeError('Missing controller consumer or conflicting motion publisher')
        path = CameraPath(); path.header.frame_id='base_link'; path.header.stamp=state['clock']
        markers = MarkerArray()
        clear=Marker(); clear.action=Marker.DELETEALL; clear.header=path.header; markers.markers.append(clear)
        for i, capture in enumerate(plan['captures']):
            matrix=np.array(capture['camera_pose']); quaternion=Rotation.from_matrix(matrix[:3,:3]).as_quat()
            pose=PoseStamped();pose.header=path.header
            pose.pose.position=Point(x=float(matrix[0,3]),y=float(matrix[1,3]),z=float(matrix[2,3]))
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,quaternion)
            arrow=Marker();arrow.header=path.header;arrow.ns='planned_view_direction';arrow.id=i;arrow.type=Marker.ARROW
            arrow.pose.orientation.w=1.;arrow.scale.x=.006;arrow.scale.y=.014;arrow.scale.z=.025
            arrow.color.g=.85;arrow.color.b=1.;arrow.color.a=.7
            target=matrix[:3,3]+.15*(fixture.target-matrix[:3,3])/np.linalg.norm(fixture.target-matrix[:3,3])
            arrow.points=[pose.pose.position,Point(x=float(target[0]),y=float(target[1]),z=float(target[2]))]
            markers.markers.append(arrow)
        # Display the FK of the actual checked joint route, not straight lines
        # between capture positions that could misleadingly cross obstacles.
        full_route = np.vstack([np.array(connector[:-1]), route])
        for a, b in zip(full_route, full_route[1:]):
            samples = max(2, int(np.ceil(max(abs(b-a))/.04))+1)
            for u in np.linspace(0., 1., samples):
                matrix = model.camera_pose(a+u*(b-a))
                quaternion = Rotation.from_matrix(matrix[:3,:3]).as_quat()
                pose = PoseStamped(); pose.header = path.header
                pose.pose.position = Point(x=float(matrix[0,3]),y=float(matrix[1,3]),z=float(matrix[2,3]))
                pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = map(float, quaternion)
                path.poses.append(pose)
        path_pub.publish(path);markers_pub.publish(markers)
        status('moving_to_route', partial=not plan['all_requested_sectors_reachable'], azimuth_span_deg=plan['azimuth_span_deg'])
        for q in connector[1:]: move(q)
        capture_by_index={c['route_index']:i for i,c in enumerate(plan['captures'])}
        for index,q in enumerate(route):
            if index: move(q)
            if index not in capture_by_index: continue
            status('capturing', view=capture_by_index[index]+1, total=len(plan['captures']))
            before=state['stats']['totals'].get('depth_integrated',0); began=time.monotonic(); deadline=began+args.dwell+8.
            while time.monotonic()<deadline:
                send(q);spin(.1)
                frames=state['stats']['totals'].get('depth_integrated',0)-before
                if time.monotonic()-began>=args.dwell and frames>=5:break
            else:raise RuntimeError('No sufficient new depth integration at this view')
            run_report['captures'].append(dict(view=capture_by_index[index]+1,
                integrated_frames=frames, dwell_wall_seconds=time.monotonic()-began,
                actual_joints=state['q'].tolist(), camera_pose=model.camera_pose(state['q']).tolist()))
        run_report.update(result='reachable_scan_finished', planned_views=len(plan['captures']),
                          all_sectors_found=plan['all_requested_sectors_reachable'],
                          azimuth_span_deg=plan['azimuth_span_deg'], measured_surface_coverage=None)
        status('reachable_scan_finished', captured_views=len(plan['captures']), azimuth_span_deg=plan['azimuth_span_deg'],
               all_sectors_reachable=plan['all_requested_sectors_reachable'], measured_surface_coverage=None)
        spin(.3)
    except KeyboardInterrupt:
        run_report['result']='stopped'
        status('stopped', reason='Ctrl+C; controller retains last position target')
    except Exception as error:
        run_report.update(result='failed', reason=str(error))
        status('failed', reason=str(error)); raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        report_name = 'last_check.json' if args.check_only else 'last_run.json'
        args.output.with_name(report_name).write_text(json.dumps(run_report, indent=2)+'\n')
        if guard is not None:guard.oracle.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
