#!/usr/bin/env python3
"""Provide truly rectified color images and matching intrinsics for aruco_ros."""
from copy import deepcopy

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def rectification_maps(info):
    if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
        raise ValueError("Expected a pinhole color camera; got " + info.distortion_model)
    intrinsic = np.asarray(info.k).reshape(3, 3)
    projection = np.asarray(info.p).reshape(3, 4)[:, :3]
    rotation = np.asarray(info.r).reshape(3, 3)
    if not np.allclose(rotation, np.eye(3)):
        raise ValueError("This color calibration path requires identity rectification rotation")
    if intrinsic[0, 0] <= 0 or projection[0, 0] <= 0:
        raise ValueError("Color CameraInfo has no valid intrinsics")
    maps = cv2.initUndistortRectifyMap(intrinsic, np.asarray(info.d), rotation,
                                     projection, (info.width, info.height), cv2.CV_32FC1)
    output = deepcopy(info)
    output.k = projection.flatten().tolist()
    output.d = [0.0]*5
    output.distortion_model = "plumb_bob"
    return maps, output


class RectifyCalibrationImage(Node):
    def __init__(self):
        super().__init__("rectify_calibration_image")
        self.bridge = CvBridge()
        self.key = self.maps = self.info = None
        self.image_pub = self.create_publisher(Image, "image_rect", 10)
        self.info_pub = self.create_publisher(CameraInfo, "rectified_camera_info", 10)
        self.create_subscription(CameraInfo, "camera_info", self.receive_info, qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self.receive_image, qos_profile_sensor_data)
        # aruco_ros skips detection if none of its output topics has subscribers;
        # a TF-only easy_handeye2 listener does not activate that lazy publisher.
        self.create_subscription(PoseStamped, "marker_pose", self.receive_marker, 10)

    def receive_marker(self, pose):
        self.get_logger().debug(f"Calibration marker received in {pose.header.frame_id}")

    def receive_info(self, info):
        key = (info.width, info.height, info.distortion_model, tuple(info.k), tuple(info.d),
               tuple(info.r), tuple(info.p), info.header.frame_id)
        if key != self.key:
            self.maps, self.info = rectification_maps(info)
            self.key = key

    def receive_image(self, image):
        if self.maps is None:
            return
        if (image.width, image.height, image.header.frame_id) != (
                self.info.width, self.info.height, self.info.header.frame_id):
            self.get_logger().error("Image and CameraInfo dimensions/frame disagree; skipping calibration image")
            return
        pixels = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        rectified = cv2.remap(pixels, *self.maps, interpolation=cv2.INTER_LINEAR)
        output = self.bridge.cv2_to_imgmsg(rectified, encoding=image.encoding)
        output.header = image.header
        info = deepcopy(self.info)
        info.header = image.header
        self.info_pub.publish(info)
        self.image_pub.publish(output)


def main():
    rclpy.init()
    node = RectifyCalibrationImage()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
