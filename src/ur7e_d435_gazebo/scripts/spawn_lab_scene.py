#!/usr/bin/env python3
"""Spawn the static lab bench; does not command robot motion."""
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
from lab_scene import build_lab_scene


def main():
    rclpy.init()
    node = Node('spawn_lab_workbench')
    share = Path(get_package_share_directory('ur7e_d435_gazebo'))
    cfg = yaml.safe_load((share/'config/lab_scene.yaml').read_text())
    xml, manifest = build_lab_scene(cfg, share/'meshes/lab')
    client = node.create_client(SpawnEntity, '/spawn_entity')
    try:
        if not client.wait_for_service(timeout_sec=90):
            raise RuntimeError('Gazebo /spawn_entity unavailable after 90 s')
        request = SpawnEntity.Request()
        request.name = 'lab_workbench'
        request.xml = xml
        request.reference_frame = 'world'
        request.initial_pose.orientation.w = 1.
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=90)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(str(future.result()) if future.done() else 'Spawn timed out')
        node.get_logger().info(f"Lab ready: {len(manifest['fixtures'])} static parts, {manifest['rack_count']} racks, {manifest['tube_count']} tubes; bench top z=0")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
