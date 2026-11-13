"""Check the generated URDF geometry, including mesh and visual-origin transforms."""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np


def origin_transform(origin):
    pose = np.eye(4)
    if origin is None:
        return pose
    roll, pitch, yaw = map(float, origin.get("rpy", "0 0 0").split())
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    pose[:3, :3] = [
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ]
    pose[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
    return pose


def link_transforms(robot, positions):
    joints = robot.findall("joint")
    children = {j.find("child").get("link") for j in joints}
    roots = {link.get("name") for link in robot.findall("link")} - children
    if len(roots) != 1:
        raise ValueError("Expected a single URDF root")
    poses = {roots.pop(): np.eye(4)}
    pending = list(joints)
    while pending:
        ready = [j for j in pending if j.find("parent").get("link") in poses]
        if not ready:
            raise ValueError("Disconnected or cyclic URDF")
        for joint in ready:
            pose = origin_transform(joint.find("origin"))
            if joint.get("type") != "fixed":
                if joint.get("type") not in ("revolute", "continuous"):
                    raise ValueError("Unsupported joint type: " + joint.get("type"))
                angle = positions[joint.get("name")]
                axis_node = joint.find("axis")
                axis = np.array(list(map(float, axis_node.get("xyz").split())))
                axis /= np.linalg.norm(axis)
                x, y, z = axis
                cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                rotation = np.eye(4)
                rotation[:3, :3] += np.sin(angle)*cross + (1-np.cos(angle))*(cross @ cross)
                pose = pose @ rotation
            poses[joint.find("child").get("link")] = poses[joint.find("parent").get("link")] @ pose
            pending.remove(joint)
    return poses


def check_mount_geometry(description, positions, executable):
    robot = ET.fromstring(description)
    poses = link_transforms(robot, positions)
    # Use tool0 coordinates to make the numbers independent of world placement.
    reference = np.linalg.inv(poses["tool0"])
    lines = []
    for link in robot.findall("link"):
        name = link.get("name")
        if name == "camera_link":
            group = "camera"
        elif name in ("base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link",
                      "wrist_1_link", "wrist_2_link", "wrist_3_link"):
            group = "arm"
        else:
            continue
        for kind in ("visual", "collision"):
            for index, element in enumerate(link.findall(kind)):
                geometry = element.find("geometry")
                mesh, box = geometry.find("mesh"), geometry.find("box")
                if mesh is not None:
                    filename = mesh.get("filename")
                    if not filename.startswith("file://"):
                        raise ValueError("Expected installed file:// mesh: " + filename)
                    path = Path(filename.removeprefix("file://"))
                    if not path.is_file():
                        raise FileNotFoundError(path)
                    shape = "mesh " + json.dumps(str(path)) + " " + mesh.get("scale", "1 1 1")
                elif box is not None:
                    shape = "box " + box.get("size")
                else:
                    raise ValueError("Unsupported mount geometry in " + name)
                pose = reference @ poses[name] @ origin_transform(element.find("origin"))
                lines.append(f"{group} {kind}:{name}:{index} {shape} " +
                             " ".join(str(value) for value in pose[:3, :].flat))
    result = subprocess.run([str(executable)], input="\n".join(lines), text=True,
                            capture_output=True, check=True, timeout=90)
    report = json.loads(result.stdout)
    report["scope"] = "Camera visual mesh and collision box vs arm visual/collision geometry at supplied joint positions"
    report["minimum_distance_m"] = min(pair["distance_m"] for pair in report["pairs"])
    report["tool0_to_color_optical"] = (reference @ poses["camera_color_optical_frame"]).tolist()
    return report
