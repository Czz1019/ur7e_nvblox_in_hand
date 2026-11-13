#!/usr/bin/env python3
"""Evaluate saved easy_handeye2 samples using the fixed base-to-marker constraint."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def matrix(transform):
    q = np.array([transform["rotation"][axis] for axis in "xyzw"], dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = [transform["translation"][axis] for axis in "xyz"]
    return result


def residuals(robot, tracking, handeye):
    marker = np.array([a @ handeye @ b for a, b in zip(robot, tracking)])
    translation = marker[:, :3, 3]
    error = np.linalg.norm(translation-translation.mean(axis=0), axis=1)
    u, _, vt = np.linalg.svd(marker[:, :3, :3].sum(axis=0))
    mean_rotation = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    angles = np.array([np.linalg.norm(cv2.Rodrigues(mean_rotation.T @ t[:3, :3])[0]) for t in marker])
    return {"translation_rms_mm": float(np.sqrt(np.mean(error**2))*1000),
            "translation_max_mm": float(error.max()*1000),
            "rotation_rms_deg": float(np.rad2deg(np.sqrt(np.mean(angles**2)))),
            "per_sample_translation_error_mm": (error*1000).tolist()}


def evaluate(samples, calibration):
    if len(samples) < 3:
        raise ValueError("At least three distinct poses are needed for evaluation")
    robot = [matrix(sample["robot"]) for sample in samples]
    tracking = [matrix(sample["tracking"]) for sample in samples]
    rotations = np.array([cv2.Rodrigues(a[:3, :3].T @ b[:3, :3])[0].ravel()
                          for i, a in enumerate(robot) for b in robot[i+1:]])
    singular = np.linalg.svd(rotations, compute_uv=False)
    report = {"sample_count": len(samples),
              "rotation_excitation_singular_values": singular.tolist(),
              "warning": "Training residuals alone do not validate calibration; use independent held-out poses and physical measurements.",
              "supplied_calibration": residuals(robot, tracking, matrix(calibration["transform"])),
              "algorithms": {}}
    for name, method in (("Tsai", cv2.CALIB_HAND_EYE_TSAI), ("Park", cv2.CALIB_HAND_EYE_PARK),
                         ("Horaud", cv2.CALIB_HAND_EYE_HORAUD), ("Andreff", cv2.CALIB_HAND_EYE_ANDREFF),
                         ("Daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS)):
        try:
            rotation, translation = cv2.calibrateHandEye(
                [t[:3, :3] for t in robot], [t[:3, 3] for t in robot],
                [t[:3, :3] for t in tracking], [t[:3, 3] for t in tracking], method=method)
            candidate = np.eye(4)
            candidate[:3, :3], candidate[:3, 3] = rotation, translation.ravel()
            if not np.isfinite(candidate).all():
                raise ValueError("Nonfinite solution; check pose diversity")
            report["algorithms"][name] = {"tool_to_camera_matrix": candidate.tolist(),
                                          **residuals(robot, tracking, candidate)}
        except (cv2.error, ValueError) as error:
            report["algorithms"][name] = {"error": str(error)}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("samples", type=Path, help="Saved easy_handeye2 .samples file")
    parser.add_argument("calibration", type=Path, help="Saved eye-in-hand .calib or YAML")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    data = yaml.safe_load(args.samples.read_text())
    calibration = yaml.safe_load(args.calibration.read_text())
    missing_parameters = []
    for key in ("calibration_type", "robot_base_frame", "robot_effector_frame", "tracking_base_frame", "tracking_marker_frame"):
        actual = data.get("parameters", {}).get(key)
        # This easy_handeye2 version saves SampleList.parameters as empty strings.
        if not actual:
            missing_parameters.append(key)
        elif actual != calibration["parameters"][key]:
            raise ValueError("Sample/calibration parameter mismatch: " + key)
    if calibration["parameters"]["calibration_type"] != "eye_in_hand":
        raise ValueError("This evaluator requires eye_in_hand samples")
    report = evaluate(data["samples"], calibration)
    if missing_parameters:
        report["sample_metadata_warning"] = (
            "Samples lack frame metadata; assumed the supplied calibration frames. "
            "Verify the original acquisition settings: " + ", ".join(missing_parameters))
    text = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
