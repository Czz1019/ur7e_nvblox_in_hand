"""Offline FK/fixture/protocol tests; no ROS node, Gazebo server or motion."""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from unittest.mock import patch
from types import SimpleNamespace

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from orbit_geometry import Model, Fixture, Guard, JOINTS, look_at, CollisionOracle
from mount_geometry import link_transforms
from orbit_scan import verify_fixture, readiness


class ScanReadinessTest(unittest.TestCase):
    def setUp(self):
        self.state = dict(q=np.zeros(6), q_wall=10., ticks=3, clock_wall=10.,
                          description='<plugin name="gazebo_ros2_control"/>',
                          info=object(), info_wall=10., stats_wall=10.,
                          stats=dict(backend='official_ros_cpp', totals={}))

    def test_all_inputs_and_individual_missing_diagnostics(self):
        self.assertEqual(readiness(self.state, 10.1)['missing'], [])
        for key, value, topic in [('q', None, '/joint_states'),
                ('ticks', 0, '/clock (advancing)'),
                ('description', '', '/robot_description (Gazebo URDF)'),
                ('info', None, '/camera/depth/camera_info'),
                ('stats', None, '/nvblox_node/metrics (native, fresh)')]:
            with self.subTest(topic=topic):
                changed = dict(self.state); changed[key] = value
                self.assertEqual(readiness(changed, 10.1)['missing'], [topic])

    def test_stale_inputs_and_incompatible_mapper_are_rejected(self):
        self.assertEqual(len(readiness(self.state, 14.)['missing']), 4)
        changed = dict(self.state, stats=dict(backend='legacy_python', totals={}))
        self.assertEqual(readiness(changed, 10.1)['missing'], ['/nvblox_node/metrics (native, fresh)'])
        changed['stats'] = []
        self.assertEqual(len(readiness(changed, 10.1)['missing']), 1)


class OrbitGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml = xacro.process_file(str(PACKAGE/'urdf/ur7e_d435.urdf.xacro')).toxml()
        cls.model = Model(cls.xml)
        cls.q = np.array([0., -1.57, 1.1, -1.57, -1.57, 0.])

    def test_fk_preserves_actual_camera_mount(self):
        for offset in [np.zeros(6), [.1,.05,-.1,.2,-.2,.1]]:
            q = self.q+offset
            expected = link_transforms(ET.fromstring(self.xml), dict(zip(JOINTS, q)))['camera_depth_optical_frame']
            np.testing.assert_allclose(self.model.camera_pose(q), expected, atol=1e-9)

    def test_look_at_optical_axis_and_handedness(self):
        p, target = np.array([-.2, .3, .8]), np.array([.4, .075, .25])
        r = look_at(p, target)
        np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(r[:,2], (target-p)/np.linalg.norm(target-p))
        self.assertAlmostEqual(np.linalg.det(r), 1.)

    def test_fcl_clear_pose_limits_and_collision(self):
        fixture = Fixture(PACKAGE/'config/collision_world_gazebo_tall.yml')
        guard = Guard(self.model, fixture)
        try:
            data = guard.oracle.query(self.q[None])
            np.testing.assert_allclose(data[:, :2], data[:, 2:])
            self.assertTrue(guard.valid(self.q))
            self.assertTrue(guard.edge(self.q, self.q))
            outside = self.q.copy(); outside[2] = self.model.upper[2]+.1
            self.assertFalse(guard.valid(outside))
            self.assertFalse(guard.edge(self.q, outside))
        finally: guard.oracle.close()
        # Deliberately place a fixture solid around the actual camera body.
        pose = self.model.poses(self.q)['camera_link'][0]
        fixture.boxes = [('blocking_box', pose[:3,3], np.array([.2,.2,.2]), np.eye(3))]
        fixture.spheres = []
        guard = Guard(self.model, fixture)
        try:
            self.assertLess(guard.gaps(self.q[None])[0][0], 0.)
            self.assertFalse(guard.valid(self.q))
        finally: guard.oracle.close()

    def test_fixture_cli_moved_model_is_rejected(self):
        fixture = Fixture(PACKAGE/'config/collision_world_gazebo_tall.yml')
        with patch('orbit_scan.subprocess.run', return_value=SimpleNamespace(stdout='0.1 0 0 0 0 0')):
            with self.assertRaisesRegex(RuntimeError, 'differs'):
                verify_fixture(fixture)

    def test_camera_is_checked_against_adjacent_wrist(self):
        pose = np.eye(4); pose[2,3] = .5
        model = SimpleNamespace(colliders=[('wrist_2_link',5,'box .1 .1 .1',np.eye(4)),
                                          ('camera_link',6,'box .1 .1 .1',np.eye(4))],
                                collider_bounds=[np.zeros(6),np.zeros(6)],motion_bound=1.,
                                poses=lambda qs: {n:np.broadcast_to(pose,(len(qs),4,4)) for n in ['wrist_2_link','camera_link']})
        fixture = SimpleNamespace(boxes=[],spheres=[('far',np.array([10.,0.,0.]),.1)])
        oracle = CollisionOracle(model,fixture)
        try:self.assertLess(oracle.gaps(np.zeros((1,6)))[1][0],0.)
        finally:oracle.close()


if __name__ == '__main__': unittest.main()
