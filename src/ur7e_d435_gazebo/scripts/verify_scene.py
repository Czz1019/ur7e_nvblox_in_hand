#!/usr/bin/env python3
"""Check actual camera messages, calibration TF, controllers and Gazebo models."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, PointCloud2
from tf2_msgs.msg import TFMessage
import yaml
from mount_geometry import check_mount_geometry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--calibration", type=Path, help="Same YAML passed to scene.launch.py")
    args = parser.parse_args()
    share = Path(get_package_share_directory("ur7e_d435_gazebo"))
    calibration_file = args.calibration or share / "config/ur7e_d435_handeye_20260907.yaml"
    calibration = yaml.safe_load(calibration_file.read_text())["transform"]
    initial = yaml.safe_load((share / "config/initial_positions.yaml").read_text())
    rclpy.init()
    node = rclpy.create_node("verify_ur7e_d435_scene")
    messages, stamps = {}, {}
    transforms = {}
    subscriptions = []
    topics = {
        "/camera/color/image_raw": (Image, "camera_color_optical_frame"),
        "/camera/depth/image_rect_raw": (Image, "camera_depth_optical_frame"),
        "/camera/depth/color/points": (PointCloud2, "camera_depth_optical_frame"),
        "/joint_states": (JointState, "base_link"),
    }
    def receive(msg, topic):
        messages[topic] = msg
        stamps.setdefault(topic, set()).add((msg.header.stamp.sec, msg.header.stamp.nanosec))
    for topic, (msg_type, _) in topics.items():
        subscriptions.append(node.create_subscription(msg_type, topic, lambda m, t=topic: receive(m, t), qos_profile_sensor_data))
    def receive_tf(msg):
        for transform in msg.transforms:
            transforms[transform.child_frame_id] = transform
    subscriptions.append(node.create_subscription(TFMessage, "/tf_static", receive_tf,
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
    parameters = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
    controller_future = parameter_future = None
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            if controller_future is None and controllers.service_is_ready():
                controller_future = controllers.call_async(ListControllers.Request())
            if parameter_future is None and parameters.service_is_ready():
                parameter_future = parameters.call_async(GetParameters.Request(names=["robot_description"]))
            rclpy.spin_once(node, timeout_sec=0.1)
            if controller_future is not None and controller_future.done():
                active = {c.name for c in controller_future.result().controller if c.state == "active"}
                if not {"joint_state_broadcaster", "forward_position_controller"} <= active:
                    controller_future = None
                    continue
                if (parameter_future is not None and parameter_future.done()
                    and all(len(stamps.get(t, set())) >= 2 for t in topics)
                    and "camera_color_optical_frame" in transforms):
                    break
        else:
            raise RuntimeError(f"Scene did not become ready; received topics: {list(messages)}")
        tf = transforms["camera_color_optical_frame"]
        assert tf.header.frame_id == "tool0", tf.header.frame_id
        tr, rot = tf.transform.translation, tf.transform.rotation
        assert all(abs(getattr(tr, axis) - calibration["translation"][axis]) < 1e-9 for axis in "xyz")
        q = [getattr(rot, axis) for axis in "xyzw"]
        wanted = [calibration["rotation"][axis] for axis in "xyzw"]
        norm = math.sqrt(sum(value*value for value in wanted))
        wanted = [value/norm for value in wanted]
        assert min(sum((a - b)**2 for a, b in zip(q, wanted)), sum((a + b)**2 for a, b in zip(q, wanted))) < 1e-12
        for topic, (_, frame) in topics.items():
            msg = messages[topic]
            assert msg.header.frame_id == frame, (topic, msg.header.frame_id)
            if topic != "/joint_states":
                assert (msg.width, msg.height) == (1280, 720), topic
                assert len(msg.data) > 0, topic
        joints = dict(zip(messages["/joint_states"].name, messages["/joint_states"].position))
        assert set(joints) == set(initial), joints
        assert all(math.isfinite(joints[j]) and abs(joints[j] - initial[j]) < 0.02 for j in initial), joints
        xml = parameter_future.result().values[0].string_value
        robot = ET.fromstring(xml)
        assert robot.find("joint[@name='tool0_camera_color_optical_joint']") is not None
        assert robot.find("link[@name='camera_bottom_screw_frame']") is not None
        for mesh in robot.findall(".//mesh"):
            filename = mesh.get("filename")
            assert filename.startswith("file://" + str(share) + "/meshes/"), filename
            assert Path(filename.removeprefix("file://")).is_file(), filename
        models = ["ur", "sage_clean_cube1", "sage_clean_cube2", "sage_clean_sphere1"]
        for name in models:
            result = subprocess.run(["gz", "model", "-m", name, "-i"], text=True, capture_output=True, timeout=15, check=True)
            assert f'name: "{name}"' in result.stdout, (name, result.stdout, result.stderr)
        geometry = check_mount_geometry(xml, joints,
            Path(get_package_prefix("ur7e_d435_gazebo")) / "lib/ur7e_d435_gazebo/check_mount_meshes")
        report = {"result": "FAIL_GEOMETRY" if geometry["collision"] else "PASS",
            "runtime_result": "PASS", "geometry": geometry,
            "calibration_file": str(calibration_file), "package_share": str(share), "models": models,
            "active_controllers": sorted(active), "transform": calibration,
            "joint_positions": joints, "topics": {t: {"frame": messages[t].header.frame_id,
                "distinct_timestamps": len(stamps[t])} for t in topics},
            "mesh_references": "All assets are installed inside this package"}
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return 1 if geometry["collision"] else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
