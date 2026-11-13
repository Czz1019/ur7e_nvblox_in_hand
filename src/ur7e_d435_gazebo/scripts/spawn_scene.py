#!/usr/bin/env python3
"""Spawn the three tall-scene obstacles from this package's installed config."""
from pathlib import Path
import threading
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import SingleThreadedExecutor
import yaml
from gazebo_obstacle_utils import spawn_gazebo_obstacles


def main():
    share = Path(get_package_share_directory("ur7e_d435_gazebo"))
    world = yaml.safe_load((share / "config/collision_world_gazebo_tall.yml").read_text())
    rclpy.init()
    node = rclpy.create_node("spawn_eih_tall_obstacles")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        ok = spawn_gazebo_obstacles(node, world, model_prefix="sage_clean", include_ground=False, service_timeout_sec=60.0)
    finally:
        executor.shutdown()
        worker.join()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
