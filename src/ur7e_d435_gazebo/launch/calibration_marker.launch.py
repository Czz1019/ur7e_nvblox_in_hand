"""Detect the calibration marker from rectified D435 color images.

Start the real camera and robot separately. Do not run another marker TF publisher.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("marker_size", description="Measured black-square side length, metres (exclude white margin)"),
        DeclareLaunchArgument("marker_id", default_value="582"),
        Node(package="ur7e_d435_gazebo", executable="rectify_calibration_image.py", output="screen",
             remappings=[("image_raw", "/camera/color/image_raw"),
                         ("camera_info", "/camera/color/camera_info"),
                         ("image_rect", "/handeye/color/image_rect"),
                         ("marker_pose", "/aruco_single/pose"),
                         ("rectified_camera_info", "/handeye/color/camera_info")]),
        Node(package="aruco_ros", executable="single", name="aruco_single", output="screen",
             parameters=[{"image_is_rectified": True,
                          "marker_size": ParameterValue(LaunchConfiguration("marker_size"), value_type=float),
                          "marker_id": ParameterValue(LaunchConfiguration("marker_id"), value_type=int),
                          "reference_frame": "camera_color_optical_frame",
                          "camera_frame": "camera_color_optical_frame",
                          "marker_frame": "camera_marker", "corner_refinement": "SUBPIX"}],
             remappings=[("/image", "/handeye/color/image_rect"),
                         ("/camera_info", "/handeye/color/camera_info")]),
    ])
