# Bundled source and asset provenance

This package snapshots the previously validated scene on 2026-09-15. It does not
require the source workspaces below at runtime.

- `urdf/ur7e_base.urdf.xacro`: expanded UR7e Gazebo description from local
  Universal_Robots_ROS2_Description 2.9.0 (`ur_type=ur7e`, safety limits enabled).
  Kinematics, joints, inertias, tool0 and geometry are unchanged. Only mesh URLs,
  initial-position YAML access and controller-config access were made package-local.
  `meshes/ur` contains the exact UR5e meshes referenced by that UR7e description.
  Source: `/home/wqj/ur_arm/ros_ur_driver/src/Universal_Robots_ROS2_Description`.
  License: `licenses/UR-Description-LICENSE`.
- `urdf/realsense` and `meshes/realsense`: Intel RealSense ROS description installed
  in `/opt/ros/humble/share/realsense2_description`. The source copyright notices
  are retained. Package asset/include paths were changed; the collision/non-mesh
  box center was subsequently aligned with the D435 visual mesh (see
  `docs/HANDEYE_DIAGNOSIS.md`). Apache-2.0.
- `third_party/realsense_gazebo_plugin`: unchanged C++ sources and headers from
  `/home/wqj/perception_D435i/src/realsense_gazebo_plugin` (package version 3.2.0).
  Built here as `librealsense_gazebo_plugin.so`, using current ROS Humble dependency
  discovery instead of the upstream build file's legacy Foxy paths.
  License: `licenses/RealSense-Apache-2.0.txt`.
- Camera mounting, side fixture, tall world, and obstacle utility were copied from
  `/home/wqj/storm/examples/SAGE_MPPI/clean_SAGE/config` and
  `/home/wqj/storm/examples/sim_gazebo`. Outer-side mounting is the accepted version:
  optical translation `[-0.0325, -0.075, 0.0357]` m, identity quaternion xyzw.

The original controller config contained several unused UR-specific controllers.
This package declares the same 100 Hz controller manager and the two controllers
actually used in the scene: joint_state_broadcaster and forward_position_controller.
