"""Offline Gazebo fixture view design. Uses the actual URDF, not a second FK model.

No ROS node, GPU or motion I/O. Scene geometry is used to check scan motion only;
it is never inserted into nvblox or presented as measured reconstruction.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import subprocess
import tempfile
import json
import time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import trimesh
import yaml
from mount_geometry import origin_transform

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def cover_box(low, high):
    low, high = np.array(low), np.array(high)
    half = (high-low)/2
    axis = int(np.argmax(half))
    n = max(1, int(np.ceil((high[axis]-low[axis])/.035)))
    centers = np.tile((low+high)/2, (n, 1))
    centers[:, axis] = low[axis]+(np.arange(n)+.5)*(high[axis]-low[axis])/n
    half[axis] /= n
    # Each sphere encloses a complete bounding-box slab, not sampled vertices.
    return centers, np.full(n, np.linalg.norm(half)+.003)


class Model:
    def __init__(self, description):
        root = ET.fromstring(description)
        self.joints = []
        pending = list(root.findall('joint'))
        children = {j.find('child').get('link') for j in pending}
        roots = {l.get('name') for l in root.findall('link')}-children
        if len(roots) != 1:
            raise ValueError('Expected one URDF root')
        self.root = roots.pop()
        owner = {self.root: 0}; known = {self.root}; limits = {}
        while pending:
            ready = [j for j in pending if j.find('parent').get('link') in known]
            if not ready: raise ValueError('Disconnected URDF')
            for j in ready:
                name, kind = j.get('name'), j.get('type')
                parent, child = j.find('parent').get('link'), j.find('child').get('link')
                if kind != 'fixed' and (kind != 'revolute' or name not in JOINTS):
                    raise ValueError('Only the six project UR joints are supported')
                axis_node = j.find('axis')
                axis = np.fromstring(axis_node.get('xyz', '1 0 0'), sep=' ') if axis_node is not None else np.array([1., 0., 0.])
                if not np.isclose(np.linalg.norm(axis), 1.): raise ValueError('Joint axis must be normalized')
                if kind != 'fixed' and owner[parent] != JOINTS.index(name):
                    raise ValueError('Collision bounds require the canonical serial UR chain')
                self.joints.append((name, parent, child, origin_transform(j.find('origin')), axis, kind))
                owner[child] = JOINTS.index(name)+1 if kind != 'fixed' else owner[parent]
                known.add(child); pending.remove(j)
                if kind != 'fixed':
                    lim, soft = j.find('limit'), j.find('safety_controller')
                    lo, hi = float(lim.get('lower')), float(lim.get('upper'))
                    if soft is not None:
                        lo = max(lo, float(soft.get('soft_lower_limit', lo)))
                        hi = min(hi, float(soft.get('soft_upper_limit', hi)))
                    limits[name] = lo+.005, hi-.005
        self.lower = np.array([limits[j][0] for j in JOINTS])
        self.upper = np.array([limits[j][1] for j in JOINTS])
        if not np.allclose(self.poses(np.zeros(6))['base_link'][0], np.eye(4), atol=1e-9):
            raise ValueError('This fixed-fixture scan expects coincident world and base_link')
        self.camera = 'camera_depth_optical_frame'
        if self.camera not in known: raise ValueError('Missing depth optical frame')
        self.camera_chain = []
        by_child = {j[2]: j for j in self.joints}
        child = self.camera
        while child != self.root:
            j = by_child[child]; self.camera_chain.insert(0, j); child = j[1]
        self.parts, self.colliders = [], []
        for link in root.findall('link'):
            name = link.get('name')
            if name == 'ground_plane': continue
            for solid in link.findall('collision'):
                geom = solid.find('geometry'); box, mesh = geom.find('box'), geom.find('mesh')
                if box is not None:
                    size = np.fromstring(box.get('size'), sep=' '); low, high = -size/2, size/2
                    kind = 'box '+' '.join(map(str, size))
                elif mesh is not None:
                    path = mesh.get('filename')
                    if not path.startswith('file://'): raise ValueError('Need resolved file:// collision meshes')
                    shape = trimesh.load_mesh(Path(path[7:]), process=False)
                    scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                    bounds = np.asarray(shape.bounds)*scale
                    low, high = bounds.min(axis=0), bounds.max(axis=0)
                    kind = 'mesh '+json.dumps(path[7:])+' '+' '.join(map(str, scale))
                else:
                    raise ValueError('Unsupported URDF collision solid')
                centers, radii = cover_box(low, high)
                t = origin_transform(solid.find('origin'))
                self.colliders.append((name, owner[name], kind, t))
                centers = centers @ t[:3, :3].T+t[:3, 3]
                self.parts.append((name, centers, radii, owner[name]))
        self.radii = np.concatenate([x[2] for x in self.parts])
        owners = np.concatenate([np.full(len(x[2]), x[3]) for x in self.parts])
        self.moving = owners > 0
        self.pairs = np.array([(i, j) for i in range(len(owners)) for j in range(i)
                               if abs(owners[i]-owners[j]) > 1], dtype=int)
        self.motion_bound = sum(np.linalg.norm(j[3][:3, 3]) for j in self.joints)+max(
            (np.linalg.norm(x[1], axis=1)+x[2]).max() for x in self.parts)
        self.collider_bounds = []
        for name, centers, radii, owner_id in self.parts:
            reach = float((np.linalg.norm(centers, axis=1)+radii).max())
            bounds = np.zeros(6); child = name
            while child != self.root:
                joint = by_child[child]
                if joint[5] != 'fixed': bounds[JOINTS.index(joint[0])] = reach
                reach += np.linalg.norm(joint[3][:3, 3]); child = joint[1]
            self.collider_bounds.append(bounds)

    @staticmethod
    def motion(q, axis):
        out = np.broadcast_to(np.eye(4), (len(q), 4, 4)).copy()
        out[:, :3, :3] = Rotation.from_rotvec(q[:, None]*axis).as_matrix()
        return out

    def camera_pose(self, q):
        pose = np.eye(4)
        for name, parent, child, origin, axis, kind in self.camera_chain:
            pose = pose @ origin
            if kind != 'fixed':
                turn = np.eye(4); turn[:3, :3] = Rotation.from_rotvec(axis*q[JOINTS.index(name)]).as_matrix()
                pose = pose @ turn
        return pose

    def poses(self, configurations):
        q = np.atleast_2d(configurations)
        poses = {self.root: np.broadcast_to(np.eye(4), (len(q), 4, 4))}
        for name, parent, child, origin, axis, kind in self.joints:
            pose = poses[parent] @ origin
            if kind != 'fixed': pose = pose @ self.motion(q[:, JOINTS.index(name)], axis)
            poses[child] = pose
        return poses

    def centers(self, configurations):
        poses = self.poses(configurations)
        return np.concatenate([local @ poses[name][:, :3, :3].transpose(0, 2, 1)+poses[name][:, None, :3, 3]
                               for name, local, radii, owner in self.parts], axis=1)


class Fixture:
    def __init__(self, filename):
        c = yaml.safe_load(Path(filename).read_text())['world_model']['coll_objs']
        self.boxes = [(name, np.array(v['pose'][:3]), np.array(v['dims'])/2,
                       Rotation.from_quat(v['pose'][3:]).as_matrix())
                      for name, v in c.get('cube', {}).items() if name != 'ground']
        self.spheres = [(name, np.array(v['position']), float(v['radius']))
                        for name, v in c.get('sphere', {}).items()]
        corners = []
        for name, center, half, rotation in self.boxes:
            corners.extend(np.array(np.meshgrid(*[[-x, x] for x in half])).reshape(3, -1).T @ rotation.T+center)
        for name, center, radius in self.spheres:
            corners.extend([center-radius, center+radius])
        points = np.array(corners); self.low, self.high = points.min(0), points.max(0)
        self.low[2] = max(0., self.low[2])
        self.target = (self.low+self.high)/2
        self.corners = np.array(np.meshgrid(*zip(self.low, self.high))).reshape(3, -1).T

    def distance(self, p):
        distances = []
        for name, center, half, rotation in self.boxes:
            d = abs((p-center) @ rotation)-half
            distances.append(np.linalg.norm(np.maximum(d, 0), axis=-1)+np.minimum(d.max(-1), 0))
        for name, center, radius in self.spheres:
            distances.append(np.linalg.norm(p-center, axis=-1)-radius)
        return np.min(distances, axis=0)


class Guard:
    def __init__(self, model, fixture, margin=.02):
        self.model, self.fixture, self.margin = model, fixture, margin
        self.oracle = CollisionOracle(model, fixture)

    def gaps(self, qs):
        return self.oracle.gaps(qs)

    def valid(self, q):
        if np.any(q < self.model.lower) or np.any(q > self.model.upper): return False
        env, own = self.gaps(np.atleast_2d(q))
        return bool(env[0] > self.margin and own[0] > .005)

    def edge(self, a, b, deadline=None):
        # Certify full joint-linear segments by FCL midpoint distances plus a
        # conservative bound on ALL collision points between samples.
        if not self.valid(np.asarray(a)) or not self.valid(np.asarray(b)): return False
        pending = [(np.asarray(a), np.asarray(b), 0)]
        while pending:
            if deadline is not None and time.monotonic() > deadline: return False
            left, right, level = pending.pop()
            midpoint = (left+right)/2
            result = self.oracle.query(midpoint[None], (right-left)/2)[0]
            if result[0] <= self.margin or result[1] <= .005: return False
            if result[2] > self.margin and result[3] > .005: continue
            if level >= 18: return False
            pending.extend([(left, midpoint, level+1), (midpoint, right, level+1)])
        return True

    def connect(self, start, goal, rng, iterations=450):
        deadline = time.monotonic()+20.
        if self.edge(start, goal, deadline): return [np.array(start), np.array(goal)]
        # Try a high, unfolded transit configuration before random search.
        # These are only proposals: all poses AND all segments are checked.
        park_a = np.array(start).copy()
        park_a[1:5] = [-math.pi/2, 0., -math.pi/2, -math.pi/2]
        park_b = park_a.copy(); park_b[0] = goal[0]; park_b[5] = goal[5]
        staging = [np.array(start), park_a, park_b, np.array(goal)]
        if (all(self.valid(q) for q in staging) and
                all(self.edge(a, b, deadline) for a, b in zip(staging, staging[1:]))):
            return staging
        # Bidirectional RRT-Connect. Most samples stay near the endpoint joint
        # branches; occasional global samples permit a larger detour.
        nodes = [[np.array(start)], [np.array(goal)]]
        parents = [[-1], [-1]]
        low = np.maximum(self.model.lower, np.minimum(start, goal)-.9)
        high = np.minimum(self.model.upper, np.maximum(start, goal)+.9)
        def extend(side, sample):
            nearest = int(np.argmin(np.linalg.norm(np.array(nodes[side])-sample, axis=1)))
            delta = sample-nodes[side][nearest]; length = np.linalg.norm(delta)
            if length < 1e-8: return nearest, True
            q = nodes[side][nearest]+delta*min(1., .35/length)
            if not self.valid(q) or not self.edge(nodes[side][nearest], q, deadline): return None, False
            nodes[side].append(q); parents[side].append(nearest)
            return len(nodes[side])-1, length <= .35
        def trace(side, index):
            route = []
            while index >= 0:
                route.append(nodes[side][index]); index = parents[side][index]
            return route[::-1]
        for i in range(iterations):
            if time.monotonic() > deadline: return None
            side = i % 2; other = 1-side
            sample = nodes[other][0] if i % 5 == 0 else (
                rng.uniform(self.model.lower, self.model.upper) if i % 8 == 0 else rng.uniform(low, high))
            index, reached = extend(side, sample)
            if index is None: continue
            for _ in range(20):
                opposite, reached = extend(other, nodes[side][index])
                if opposite is None: break
                if reached:
                    ids = {side: index, other: opposite}
                    return trace(0, ids[0])+trace(1, ids[1])[-2::-1]
        return None


def look_at(position, target):
    z = target-position; z /= np.linalg.norm(z)
    x = np.cross(z, [0., 0., 1.]); x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


class CollisionOracle:
    def __init__(self, model, fixture):
        self.model = model
        binary = Path(__file__).resolve().parents[3]/'install/ur7e_d435_gazebo/lib/ur7e_d435_gazebo/scan_collision_checker'
        if not binary.is_file(): raise FileNotFoundError('Run ./build.sh to build the scan collision checker')
        self.errors = tempfile.TemporaryFile(mode='w+t')
        self.process = subprocess.Popen([str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.errors, text=True, bufsize=1)
        identity = np.eye(4)
        def row(owner, kind, pose, bounds=None, payload=False):
            if bounds is None: bounds = np.zeros(6)
            return f'{owner} {kind} '+' '.join(map(str, pose[:3].flat))+' '+' '.join(map(str, bounds))+f' {int(payload)}\n'
        rows = [f'{len(model.colliders)} {1+len(fixture.boxes)+len(fixture.spheres)} {model.motion_bound}\n']
        for collider, bounds in zip(model.colliders, model.collider_bounds):
            name, owner, kind, origin = collider
            rows.append(row(owner, kind, identity, bounds, name.startswith('camera_')))
        rows.append(row(-2, 'floor', identity))
        for name, center, half, rotation in fixture.boxes:
            pose = identity.copy(); pose[:3, :3] = rotation; pose[:3, 3] = center
            rows.append(row(-1, 'box '+' '.join(map(str, half*2)), pose))
        for name, center, radius in fixture.spheres:
            pose = identity.copy(); pose[:3, 3] = center
            rows.append(row(-1, 'sphere '+str(radius), pose))
        self.process.stdin.write(''.join(rows)); self.process.stdin.flush()
        if self.process.stdout.readline().strip() != 'READY':
            self.errors.seek(0); raise RuntimeError('FCL initialization failed: '+self.errors.read())

    def query(self, qs, half_delta=None):
        qs = np.atleast_2d(qs); results = []
        poses = self.model.poses(qs)
        solids = np.stack([poses[name] @ origin for name, owner, kind, origin in self.model.colliders], axis=1)
        for start in range(0, len(qs), 64):
            batch = solids[start:start+64, :, :3, :]
            rows = [str(len(batch))+'\n']
            delta = np.zeros(6) if half_delta is None else np.asarray(half_delta)
            rows += [' '.join(map(str, delta))+' '+' '.join(map(str, pose.flat))+'\n' for pose in batch]
            self.process.stdin.write(''.join(rows)); self.process.stdin.flush()
            for _ in batch:
                line = self.process.stdout.readline()
                if not line: raise RuntimeError('FCL collision checker exited')
                values = list(map(float, line.split()))
                if len(values) != 4 or not np.isfinite(values).all(): raise RuntimeError('Invalid FCL result')
                results.append(values)
        return np.array(results)

    def gaps(self, qs):
        result = self.query(qs)
        return result[:, 0], result[:, 1]

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try: self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired: self.process.kill(); self.process.wait()
        self.errors.close()
        self.process.stdout.close()

    def __del__(self):
        if hasattr(self, 'process'):
            try: self.close()
            except Exception: pass


def design(model, fixture, start, views=12, progress=lambda s: None):
    if views < 6 or views > 24: raise ValueError('views must be between 6 and 24')
    guard = Guard(model, fixture)
    env, own = guard.gaps(np.atleast_2d(start))
    if not guard.valid(start):
        raise ValueError(f'Start fails conservative URDF collision cover: environment={env[0]:.4f}, self={own[0]:.4f} m')
    rng = np.random.default_rng(127)
    current = np.array(start); route = [current.tolist()]; captures = []; rejected = []
    first_pose = model.camera_pose(current)
    first_angle = math.atan2(first_pose[1, 3]-fixture.target[1], first_pose[0, 3]-fixture.target[0])+2*math.pi/3
    sectors = [(height, index) for ring, height in enumerate([.35, .6, .8])
               for index in (range(views) if ring == 0 else reversed(range(views)))]
    for slot, (height, index) in enumerate(sectors):
        angle = first_angle+index*2*math.pi/views
        candidates = []
        for height in [height]:
            for radius in [.35, .45, .55, .65, .75]:
                position = fixture.target+np.array([radius*math.cos(angle), radius*math.sin(angle), height-fixture.target[2]])
                rotation = look_at(position, fixture.target)
                upright = rotation.copy()
                local = (fixture.corners-position) @ rotation
                framed = (local[:, 2] > .2) & (abs(local[:, 0]) < local[:, 2]*.93) & (abs(local[:, 1]) < local[:, 2]*.53)
                if framed.sum() < (2 if radius <= .45 and height <= .6 else 4): continue
                # Alternative elbow branches are IK initialization only, not executable waypoints.
                seeds = [current, np.array([0., -1.57, 1.1, -.91, -3.21, -.69]),
                         np.array([.91, -.036, -1.06, -.978, 2.025, -1.689]),
                         np.array([-3.082, -2.521, -1.01, -.925, -1.689, -2.80])]
                seeds += [rng.uniform(model.lower+.01, model.upper-.01) for _ in range(2)]
                for attempt, seed in enumerate(seeds+seeds[:2]):
                    # Allow a 180-degree optical roll when upright IK is
                    # restricted. Optical +Z still faces the same target.
                    rotation = upright if attempt < len(seeds) else upright @ np.diag([-1., -1., 1.])
                    def residual(q):
                        pose = model.camera_pose(q)
                        return np.r_[(pose[:3, 3]-position)/.05,
                                     Rotation.from_matrix(pose[:3, :3].T @ rotation).as_rotvec()/.2]
                    solution = least_squares(residual, np.clip(seed, model.lower+.001, model.upper-.001),
                                             bounds=(model.lower, model.upper), max_nfev=70, ftol=1e-5)
                    q = solution.x
                    # Choose the equivalent joint angles closest to the current branch.
                    for j in range(6):
                        options = [q[j]+2*math.pi*k for k in [-1, 0, 1]]
                        options = [x for x in options if model.lower[j] <= x <= model.upper[j]]
                        q[j] = min(options, key=lambda x: abs(x-current[j]))
                    pose = model.camera_pose(q)
                    if (np.linalg.norm(pose[:3, 3]-position) > .012 or
                            np.linalg.norm(Rotation.from_matrix(pose[:3, :3].T @ rotation).as_rotvec()) > .05 or
                            not guard.valid(q)):
                        continue
                    if any(np.linalg.norm(q-v[1]) < .03 for v in candidates): continue
                    candidates.append((np.linalg.norm(q-current), q.copy(), int(framed.sum())))
        connected = None
        for cost, q, framed in sorted(candidates, key=lambda x: x[0])[:3]:
            connected = guard.connect(current, q, rng)
            if connected is not None: break
        if connected is None:
            rejected.append({'azimuth_index': index, 'azimuth_deg': math.degrees(angle)%360,
                             'reason': 'IK/framing/collision/connection did not yield a feasible view'})
            progress(f'View {slot+1}/{len(sectors)} (z={height:.2f}): unavailable')
            continue
        route.extend([q.tolist() for q in connected[1:]])
        current = connected[-1]
        captures.append({'route_index': len(route)-1, 'azimuth_deg': math.degrees(angle)%360,
                         'framed_bbox_corners': framed, 'camera_pose': model.camera_pose(current).tolist()})
        progress(f'View {slot+1}/{len(sectors)} (z={height:.2f}): accepted ({framed}/8 bbox corners in conservative FOV)')
    if len(captures) < 3:
        raise ValueError(f'Only {len(captures)} reachable scan views; refusing a purported overall scan')
    angles = sorted(c['azimuth_deg'] for c in captures)
    span = 360-max(np.diff(angles+[angles[0]+360]))
    guard.oracle.close()
    return {'schema': 1, 'planner_revision': 4, 'frame': 'base_link', 'joint_names': JOINTS, 'target': fixture.target.tolist(),
            'target_bounds': [fixture.low.tolist(), fixture.high.tolist()], 'route': route,
            'captures': captures, 'rejected_sectors': rejected, 'requested_sectors': len(sectors), 'height_levels_m': [.35, .6, .8],
            'accepted_sectors': len(captures), 'azimuth_span_deg': float(span),
            'all_requested_sectors_reachable': not rejected,
            'collision_check': 'FCL actual URDF collision meshes/boxes, exact vertex floor height, configured primitive fixture; adaptive edge motion bound',
            'environment_margin_m': .02, 'self_margin_m': .005,
            'geometry_inserted_into_map': False, 'visible_surface_coverage_measured': False}
