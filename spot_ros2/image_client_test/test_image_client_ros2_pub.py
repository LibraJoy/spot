#!/usr/bin/env python3
"""
Spot CAM+ Single Camera ROS2 Publisher
Publishes image and camera info for a single camera source
"""

from bosdyn.client.image import build_image_request, ImageClient
import bosdyn.client.spot_cam as spot_cam
import bosdyn.client.util
from spot_ros2.CameraService import CameraService
import cv2
import numpy as np
import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import time


class SpotCameraPublisher(Node):
    def __init__(self, source='c2', resize_ratio=0.30, quality_percent=100, image_format=2):
        super().__init__('spot_camera_publisher')

        # =================================================================
        # CONFIGURATION
        # =================================================================

        self.source = source  # Camera source: 'c0', 'c1', 'c2', 'c3', 'c4', 'pano', etc.
        self.resize_ratio = resize_ratio
        self.quality_percent = quality_percent
        self.image_format = image_format  # 1=JPEG, 2=RAW

        self.get_logger().info(f"Starting Spot Camera Publisher:")
        self.get_logger().info(f"  Source: {source}")
        self.get_logger().info(f"  Resize ratio: {resize_ratio}")
        self.get_logger().info(f"  Quality: {quality_percent}%")
        self.get_logger().info(f"  Format: {'JPEG' if image_format == 1 else 'RAW'}")

        # =================================================================
        # LOAD CAMERA CALIBRATION
        # =================================================================

        # Try to load calibration from JSON file
        calib_file = f'{source}_fisheye_calibration.json'
        if os.path.exists(calib_file):
            self.get_logger().info(f"Loading calibration from {calib_file}")
            with open(calib_file, 'r') as f:
                calib = json.load(f)

            # Extract intrinsics (at original resolution)
            self.fx_orig = calib['fx']
            self.fy_orig = calib['fy']
            self.cx_orig = calib['cx']
            self.cy_orig = calib['cy']

            # Extract distortion coefficients
            self.k1 = calib['k1']
            self.k2 = calib['k2']
            self.k3 = calib['k3']
            self.k4 = calib['k4']

            self.get_logger().info(f"  Loaded calibration: fx={self.fx_orig:.2f}, k1={self.k1:.6f}")

        else:
            # Use default intrinsics (from camera_paramters.txt)
            self.get_logger().warn(f"Calibration file {calib_file} not found, using defaults")

            defaults = {
                # 'c0': {'fx': 731.103, 'fy': 730.925, 'cx': 966.709, 'cy': 573.586},
                # 'c1': {'fx': 732.681, 'fy': 732.484, 'cx': 991.069, 'cy': 555.718},
                # 'c2': {'fx': 731.200, 'fy': 730.983, 'cx': 1004.887, 'cy': 592.168},
                # 'c3': {'fx': 731.102, 'fy': 730.912, 'cx': 915.568, 'cy': 569.939},
                # 'c4': {'fx': 732.784, 'fy': 732.587, 'cx': 957.771, 'cy': 552.834},
            }

            if source in defaults:
                self.fx_orig = defaults[source]['fx']
                self.fy_orig = defaults[source]['fy']
                self.cx_orig = defaults[source]['cx']
                self.cy_orig = defaults[source]['cy']
            else:
                # Generic defaults
                self.fx_orig = 730.0
                self.fy_orig = 730.0
                self.cx_orig = 960.0
                self.cy_orig = 540.0

            # No distortion coefficients
            self.k1 = 0.0
            self.k2 = 0.0
            self.k3 = 0.0
            self.k4 = 0.0

        # Scale intrinsics by resize ratio
        self.fx = self.fx_orig * resize_ratio
        self.fy = self.fy_orig * resize_ratio
        self.cx = self.cx_orig * resize_ratio
        self.cy = self.cy_orig * resize_ratio

        self.get_logger().info(f"  Scaled intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}")

        # =================================================================
        # ROS2 PUBLISHERS
        # =================================================================

        self.image_pub = self.create_publisher(Image, '/image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)

        self.bridge = CvBridge()

        # Performance monitoring
        self.last_time = time.time()
        self.fps_counter = 0

        # =================================================================
        # CONNECT TO SPOT
        # =================================================================

        self.get_logger().info("Connecting to Spot...")
        bosdyn.client.util.setup_logging(False)
        sdk = bosdyn.client.create_standard_sdk('SpotCameraPublisher')
        spot_cam.register_all_service_clients(sdk)

        robot = sdk.create_robot('10.0.0.3')
        robot.authenticate('user', 'scgau6g5w987')

        self.ptzCam = CameraService(robot)
        self.cam_image_client = robot.ensure_client("spot-cam-image")

        self.get_logger().info("Connected to Spot CAM+")

        # =================================================================
        # BUILD IMAGE REQUEST
        # =================================================================

        self.image_request = [
            build_image_request(
                self.source,
                quality_percent=self.quality_percent,
                resize_ratio=self.resize_ratio,
                image_format=self.image_format
            )
        ]

        # Get first image to determine actual resolution
        first_response = self.cam_image_client.get_image(self.image_request)
        first_img = first_response[0].shot.image

        self.image_width = first_img.cols
        self.image_height = first_img.rows

        self.get_logger().info(f"Actual image size: {self.image_width}x{self.image_height}")

        # =================================================================
        # CREATE TIMER FOR PUBLISHING
        # =================================================================

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_callback)

        self.frame_count = 0
        self.get_logger().info("Publisher started!")

    def publish_callback(self):
        """Main publishing loop - called at 10 Hz"""

        try:
            t0 = time.time()

            # =========================================================
            # CAPTURE IMAGE - OPTIMIZED
            # =========================================================

            image_responses = self.cam_image_client.get_image(self.image_request)
            response = image_responses[0]
            img_proto = response.shot.image

            t1 = time.time()

            # =========================================================
            # PUBLISH IMAGE - OPTIMIZED (Skip cv_bridge overhead)
            # =========================================================

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.source

            # Build ROS Image message directly without cv_bridge conversion
            image_msg = Image()
            image_msg.header = header
            image_msg.height = img_proto.rows
            image_msg.width = img_proto.cols
            image_msg.encoding = 'rgb8'  # RAW format is already RGB
            image_msg.is_bigendian = 0
            image_msg.step = img_proto.cols * 3  # bytes per row

            if img_proto.format == 2:  # RAW
                # Direct copy - no conversion needed!
                image_msg.data = img_proto.data
            else:  # JPEG
                # Decode JPEG to RGB
                np_img = cv2.imdecode(
                    np.frombuffer(img_proto.data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )
                # Convert BGR to RGB
                np_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
                image_msg.data = np_img.tobytes()

            t2 = time.time()

            self.image_pub.publish(image_msg)

            t3 = time.time()

            # =========================================================
            # PUBLISH CAMERA INFO
            # =========================================================

            camera_info = CameraInfo()
            camera_info.header = header

            # Image dimensions
            camera_info.height = self.image_height
            camera_info.width = self.image_width

            # Distortion model - use 'equidistant' for fisheye cameras
            camera_info.distortion_model = 'equidistant'

            # Camera matrix K [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            camera_info.k = [
                self.fx, 0.0, self.cx,
                0.0, self.fy, self.cy,
                0.0, 0.0, 1.0
            ]

            # Distortion coefficients D [k1, k2, k3, k4] for equidistant model
            camera_info.d = [
                float(self.k1),
                float(self.k2),
                float(self.k3),
                float(self.k4)
            ]

            # Rectification matrix R (identity for unrectified)
            camera_info.r = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]

            # Projection matrix P [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]
            camera_info.p = [
                self.fx, 0.0, self.cx, 0.0,
                0.0, self.fy, self.cy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]

            self.camera_info_pub.publish(camera_info)

            # =========================================================
            # LOG STATUS WITH PERFORMANCE METRICS
            # =========================================================

            self.frame_count += 1
            self.fps_counter += 1

            # Calculate actual FPS every 30 frames
            if self.frame_count % 100 == 0:
                elapsed = time.time() - self.last_time
                actual_fps = self.fps_counter / elapsed

                # Timing breakdown
                capture_time = (t1 - t0) * 1000  # ms
                convert_time = (t2 - t1) * 1000  # ms
                publish_time = (t3 - t2) * 1000  # ms
                total_time = (t3 - t0) * 1000    # ms

                self.get_logger().info(
                    f"Frame {self.frame_count} | "
                    f"FPS: {actual_fps:.2f} Hz | "
                    f"Times(ms): capture={capture_time:.1f}, "
                    f"convert={convert_time:.1f}, publish={publish_time:.1f}, "
                    f"total={total_time:.1f}"
                )

                # Reset FPS counter
                self.last_time = time.time()
                self.fps_counter = 0

        except Exception as e:
            self.get_logger().error(f"Error in publish_callback: {e}")


def main(args=None):
    rclpy.init(args=args)

    # =================================================================
    # CONFIGURATION - CHANGE THESE SETTINGS
    # =================================================================

    SOURCE = 'c2'              # Camera source: 'c0', 'c1', 'c2', 'c3', 'c4', 'pano', etc.
    RESIZE_RATIO = 0.30        # 0.3 = 30% of original size (576x324 for fisheye)
    QUALITY_PERCENT = 100      # JPEG quality 0-100 (only applies if image_format=1)
    IMAGE_FORMAT = 2           # 1=JPEG, 2=RAW

    # =================================================================

    node = SpotCameraPublisher(
        source=SOURCE,
        resize_ratio=RESIZE_RATIO,
        quality_percent=QUALITY_PERCENT,
        image_format=IMAGE_FORMAT
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
