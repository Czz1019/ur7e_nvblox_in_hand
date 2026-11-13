"""Run after sourcing ROS and the workspace: python3 -m unittest discover -s .../test."""
from pathlib import Path
from copy import deepcopy
import json
import importlib.util
import sys
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo
import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from evaluate_handeye import evaluate, matrix, residuals
from mount_geometry import check_mount_geometry, link_transforms, origin_transform
from rectify_calibration_image import rectification_maps
from import_handeye_20260907 import convert


class GazeboStartupTest(unittest.TestCase):
    def test_occupied_master_is_rejected_before_launching_nodes(self):
        path = Path(__file__).resolve().parents[1] / "launch/scene.launch.py"
        spec = importlib.util.spec_from_file_location("scene_launch", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        connection = Mock()
        with patch.object(module.socket, "create_connection", return_value=connection):
            with self.assertRaisesRegex(RuntimeError, "already in use"):
                module._check_gazebo_master_available("http://127.0.0.1:11345")
        connection.close.assert_called_once()
        with patch.object(module.socket, "create_connection", side_effect=ConnectionRefusedError):
            module._check_gazebo_master_available("http://127.0.0.1:11345")


class HandeyeGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.share = Path(get_package_share_directory("ur7e_d435_gazebo"))
        cls.positions = yaml.safe_load((cls.share / "config/initial_positions.yaml").read_text())
        cls.checker = Path(get_package_prefix("ur7e_d435_gazebo")) / "lib/ur7e_d435_gazebo/check_mount_meshes"

    def description(self, config):
        return xacro.process_file(str(self.share / "urdf/ur7e_d435.urdf.xacro"),
            mappings={"eih_calibration": str(self.share / "config" / config)}).toxml()

    def test_measured_transform_and_camera_internal_geometry(self):
        robot = ET.fromstring(self.description("ur7e_d435_side_sim_mount.yaml"))
        poses = link_transforms(robot, self.positions)
        calibration = yaml.safe_load((self.share / "config/ur7e_d435_side_sim_mount.yaml").read_text())
        np.testing.assert_allclose(np.linalg.inv(poses["tool0"]) @ poses["camera_color_optical_frame"],
                                   matrix(calibration["transform"]), atol=1e-12)
        color_from_link = origin_transform(ET.fromstring('<origin xyz="0 .015 0" rpy="-1.5707963267948966 0 -1.5707963267948966"/>'))
        np.testing.assert_allclose(poses["camera_link"] @ color_from_link,
                                   poses["camera_color_optical_frame"], atol=1e-12)
        collision = robot.find("link[@name='camera_link']/collision")
        center = origin_transform(collision.find("origin"))[:3, 3]
        size = np.array(list(map(float, collision.find("geometry/box").get("size").split())))
        # The DAE front plate is at +4.3 mm; housing depth is 25.05 mm.
        np.testing.assert_allclose([center[0]-size[0]/2, center[0]+size[0]/2], [-.02075, .0043], atol=1e-12)

    def test_measured_overlap_is_reported(self):
        report = check_mount_geometry(self.description("ur7e_d435_side_sim_mount.yaml"), self.positions, self.checker)
        self.assertTrue(report["collision"])
        self.assertTrue(any(p["collision"] and p["arm"] == "visual:wrist_3_link:0" for p in report["pairs"]))

    def test_explicit_simulation_mount_has_clearance(self):
        report = check_mount_geometry(self.description("ur7e_d435_sim_clearance.yaml"), self.positions, self.checker)
        self.assertFalse(report["collision"])
        self.assertGreater(report["minimum_distance_m"], .03)

    def test_default_charuco_mount_matches_source_and_has_clearance(self):
        xml = xacro.process_file(str(self.share / "urdf/ur7e_d435.urdf.xacro")).toxml()
        poses = link_transforms(ET.fromstring(xml), self.positions)
        source = json.loads((self.share / "config/handeye_20260907/calibration_20260907.json").read_text())
        np.testing.assert_allclose(np.linalg.inv(poses["tool0"]) @ poses["camera_color_optical_frame"],
                                   source["hand_eye"]["gripper_T_camera"], atol=1e-8, rtol=0)
        report = check_mount_geometry(xml, self.positions, self.checker)
        self.assertFalse(report["collision"])
        self.assertGreater(report["minimum_distance_m"], .012)


class CalibrationImportTest(unittest.TestCase):
    def setUp(self):
        config = Path(__file__).resolve().parents[1] / "config"
        self.legacy = yaml.safe_load((config / "handeye_20260907/legacy_calibration.yaml").read_text())
        self.report = json.loads((config / "handeye_20260907/calibration_20260907.json").read_text())

    def test_units_and_quaternion_preserve_matrix(self):
        imported = convert(self.legacy, self.report)
        np.testing.assert_allclose(matrix(imported["transform"]), self.report["hand_eye"]["gripper_T_camera"], atol=1e-8, rtol=0)
        self.assertAlmostEqual(imported["transform"]["translation"]["x"], -.029901376)
        self.assertEqual(imported["calibration_provenance"]["global_corner_rmse_px"], 2.1793760746)

    def test_wrong_units_and_mismatched_files_are_rejected(self):
        wrong = deepcopy(self.report)
        wrong["hand_eye"]["translation_unit"] = "millimetre"
        with self.assertRaisesRegex(ValueError, "metres"):
            convert(self.legacy, wrong)
        wrong = deepcopy(self.legacy)
        wrong["gripper_T_camera"][0][3] /= 1000
        with self.assertRaisesRegex(ValueError, "disagree"):
            convert(wrong, self.report)

    def test_reference_intrinsics_are_cross_checked(self):
        wrong = deepcopy(self.legacy)
        wrong["camera_matrix"][0][0] += 10
        with self.assertRaisesRegex(ValueError, "intrinsics disagree"):
            convert(wrong, self.report)


class CalibrationInputTest(unittest.TestCase):
    def test_rectified_intrinsics_and_distortion(self):
        info = CameraInfo()
        info.width, info.height = 64, 48
        info.distortion_model = "plumb_bob"
        info.k = [70., 0., 32., 0., 71., 24., 0., 0., 1.]
        info.r = np.eye(3).flatten().tolist()
        info.p = [68., 0., 31., 0., 0., 69., 23., 0., 0., 0., 1., 0.]
        info.d = [.12, -.03, .001, .002, .005]
        maps, rectified = rectification_maps(info)
        np.testing.assert_allclose(np.asarray(rectified.k).reshape(3, 3), np.asarray(info.p).reshape(3, 4)[:, :3])
        self.assertTrue(all(value == 0 for value in rectified.d))
        np.testing.assert_allclose(info.d, [.12, -.03, .001, .002, .005])
        # Rectified output pixel -> distorted raw source -> rectified pixel must close.
        points = np.array([[[float(maps[0][8, 9]), float(maps[1][8, 9])]]])
        recovered = cv2.undistortPoints(points, np.asarray(info.k).reshape(3, 3), np.asarray(info.d),
                                       P=np.asarray(rectified.k).reshape(3, 3))
        np.testing.assert_allclose(recovered[0, 0], [9, 8], atol=1e-4)

    def test_fixed_marker_residual_detects_bad_calibration(self):
        rng = np.random.default_rng(4)
        handeye = np.eye(4)
        handeye[:3, :3] = cv2.Rodrigues(np.array([.2, -.4, .3]))[0]
        handeye[:3, 3] = [.03, -.1, .02]
        base_marker = np.eye(4)
        base_marker[:3, 3] = [.5, .1, .4]
        robot, tracking = [], []
        def msg(t):
            # Synthetic poses use rotation matrices converted through axis-angle.
            axis_angle = cv2.Rodrigues(t[:3, :3])[0].ravel()
            angle = np.linalg.norm(axis_angle)
            q = np.r_[axis_angle/angle*np.sin(angle/2), np.cos(angle/2)] if angle else np.array([0., 0., 0., 1.])
            return {"translation": dict(zip("xyz", map(float, t[:3, 3]))),
                    "rotation": dict(zip("xyzw", map(float, q)))}
        for _ in range(20):
            pose = np.eye(4)
            pose[:3, :3] = cv2.Rodrigues(rng.normal(0, .4, 3))[0]
            pose[:3, 3] = rng.normal(0, .1, 3)
            robot.append(pose)
            tracking.append(np.linalg.inv(handeye) @ np.linalg.inv(pose) @ base_marker)
        report = evaluate([{"robot": msg(a), "tracking": msg(b)} for a, b in zip(robot, tracking)],
                          {"transform": msg(handeye)})
        self.assertLess(report["supplied_calibration"]["translation_rms_mm"], 1e-8)
        np.testing.assert_allclose(report["algorithms"]["Park"]["tool_to_camera_matrix"], handeye, atol=1e-8)
        wrong = handeye.copy()
        wrong[0, 3] += .02
        self.assertGreater(residuals(robot, tracking, wrong)["translation_rms_mm"], 1.)


if __name__ == "__main__":
    unittest.main()
