#!/usr/bin/env python3
"""Cross-check the supplied mm YAML and metre JSON, then export ROS hand-eye YAML."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def convert(legacy, report):
    if report["hand_eye"]["translation_unit"] != "metre":
        raise ValueError("JSON hand-eye translation must be in metres")
    original = np.asarray(report["hand_eye"]["gripper_T_camera"], dtype=float)
    millimetres = np.asarray(legacy["gripper_T_camera"], dtype=float).copy()
    if original.shape != (4, 4) or millimetres.shape != (4, 4):
        raise ValueError("Expected 4x4 gripper_T_camera matrices")
    millimetres[:3, 3] *= .001
    if not np.isfinite(original).all() or not np.allclose(original, millimetres, atol=1e-6, rtol=0):
        raise ValueError("YAML (mm) and JSON (m) hand-eye matrices disagree")
    if not np.allclose(original[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError("Invalid homogeneous matrix last row")
    rotation = original[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-6, rtol=0) or np.linalg.det(rotation) <= 0:
        raise ValueError("Expected a proper rotation, allowing only export rounding")
    # Remove decimal-export roundoff, not a physical adjustment of the calibration.
    u, _, vt = np.linalg.svd(rotation)
    rotation = u @ vt
    vector = cv2.Rodrigues(rotation)[0].ravel()
    angle = np.linalg.norm(vector)
    quaternion = np.r_[vector/angle*np.sin(angle/2), np.cos(angle/2)] if angle else np.array([0., 0., 0., 1.])
    intrinsics = report["runtime_intrinsics"]
    if (legacy["legacy_color_resolution"] != intrinsics["resolution"] or
            not np.allclose(legacy["camera_matrix"], intrinsics["camera_matrix"], atol=1e-6, rtol=0) or
            not np.allclose(np.asarray(legacy["distortion_coefficients"]).ravel(), intrinsics["distortion"], atol=1e-9, rtol=0)):
        raise ValueError("YAML and JSON color intrinsics disagree")
    return {
        "parameters": {"name": "charuco_20260907", "calibration_type": "eye_in_hand",
                       "robot_base_frame": "base_link", "robot_effector_frame": "tool0",
                       "tracking_base_frame": "camera_color_optical_frame"},
        "transform": {"translation": dict(zip("xyz", map(float, original[:3, 3]))),
                      "rotation": dict(zip("xyzw", map(float, quaternion)))},
        "calibration_provenance": {
            "source": "handeye_20260907/calibration_20260907.json",
            "cross_checked_source": "handeye_20260907/legacy_calibration.yaml",
            "gripper_frame": "tool0 (confirmed by user)",
            "camera_frame_assumption": "color optical frame, inferred from color ChArUco reprojection calibration",
            "method": report["hand_eye"]["method"],
            "accepted_images": report["accepted_images"],
            "global_corner_rmse_px": report["hand_eye"]["global_corner_rmse_px"],
            "rotation_roundoff_correction_max": float(np.max(np.abs(rotation-original[:3, :3]))),
            "board": report["board"],
        },
        # Reference only: do not apply 1920x1080 color K to 1280x720 depth images.
        "reference_color_intrinsics": intrinsics,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("legacy_yaml", type=Path)
    parser.add_argument("calibration_json", type=Path)
    parser.add_argument("output_yaml", type=Path)
    args = parser.parse_args()
    converted = convert(yaml.safe_load(args.legacy_yaml.read_text()), json.loads(args.calibration_json.read_text()))
    args.output_yaml.write_text("# Measured ChArUco calibration; translation in metres, quaternion xyzw.\n" +
                                yaml.safe_dump(converted, sort_keys=False, allow_unicode=True))
    print(yaml.safe_dump(converted["transform"], sort_keys=False))


if __name__ == "__main__":
    main()
