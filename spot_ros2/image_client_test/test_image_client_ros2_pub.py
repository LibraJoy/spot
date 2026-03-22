#!/usr/bin/env python3
"""
Spot CAM+ Panoramic Image ROS2 Publisher
Captures 'pano' image, splits into 5 fisheye cameras, calibrates, stitches, and publishes
"""

from bosdyn.client.image import build_image_request, ImageClient
import bosdyn.client.spot_cam as spot_cam
import bosdyn.client.util
from spot_ros2.CameraService import CameraService
import cv2
import numpy as np
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge

class SpotPanoPublisher(Node):
    def __init__(self, resize_ratio=0.30, quality_percent=100, image_format=2, pixel_format=3):
        super().__init__('spot_pano_publisher')

        # Image capture parameters (adjustable)
        self.resize_ratio = resize_ratio
        self.quality_percent = quality_percent
        self.image_format = image_format
        self.pixel_format = pixel_format

        # Original resolution: 1920x1080 per camera, 9600x1080 for pano
        self.original_camera_width = 1920
        self.original_camera_height = 1080

        # Resized resolution (with resize_ratio applied)
        self.camera_width = int(self.original_camera_width * resize_ratio)  # 576
        self.camera_height = int(self.original_camera_height * resize_ratio)  # 324
        self.pano_width = self.camera_width * 5  # 2880
        self.pano_height = self.camera_height  # 324

        self.get_logger().info(f"Resolution: {self.camera_width}x{self.camera_height} per camera, "
                               f"{self.pano_width}x{self.pano_height} pano")

        # ROS2 publishers
        self.pano_pub = self.create_publisher(Image, '/spot_image_pano', 10)
        self.pano_params_pub = self.create_publisher(TransformStamped, '/pano_parameters', 10)
        self.camera_params_pub = self.create_publisher(CameraInfo, '/camera_parameters', 10)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Initialize Spot connection
        self.get_logger().info("Connecting to Spot...")
        bosdyn.client.util.setup_logging(False)
        sdk = bosdyn.client.create_standard_sdk('SpotPanoPublisher')
        spot_cam.register_all_service_clients(sdk)

        robot = sdk.create_robot('10.0.0.3')
        robot.authenticate('user', 'scgau6g5w987')

        self.ptzCam = CameraService(robot)
        self.cam_image_client = robot.ensure_client("spot-cam-image")

        self.get_logger().info("Connected to Spot CAM+")

        # Camera intrinsics for c0-c4 (from camera_paramters.txt, scaled by resize_ratio)
        # Original intrinsics are at 1920x1080, scale to resized resolution
        self.camera_intrinsics = {
            'c0': {
                'fx': 731.103 * resize_ratio, 'fy': 730.925 * resize_ratio,
                'cx': 966.709 * resize_ratio, 'cy': 573.586 * resize_ratio,
                'k1': 0, 'k2': 0, 'p1': 0, 'p2': 0
            },
            'c1': {
                'fx': 732.681 * resize_ratio, 'fy': 732.484 * resize_ratio,
                'cx': 991.069 * resize_ratio, 'cy': 555.718 * resize_ratio,
                'k1': 0, 'k2': 0, 'p1': 0, 'p2': 0
            },
            'c2': {
                'fx': 731.200 * resize_ratio, 'fy': 730.983 * resize_ratio,
                'cx': 1004.887 * resize_ratio, 'cy': 592.168 * resize_ratio,
                'k1': 0, 'k2': 0, 'p1': 0, 'p2': 0
            },
            'c3': {
                'fx': 731.102 * resize_ratio, 'fy': 730.912 * resize_ratio,
                'cx': 915.568 * resize_ratio, 'cy': 569.939 * resize_ratio,
                'k1': 0, 'k2': 0, 'p1': 0, 'p2': 0
            },
            'c4': {
                'fx': 732.784 * resize_ratio, 'fy': 732.587 * resize_ratio,
                'cx': 957.771 * resize_ratio, 'cy': 552.834 * resize_ratio,
                'k1': 0, 'k2': 0, 'p1': 0, 'p2': 0
            },
        }

        # Camera extrinsics (relative to spot_cam_payload_frame, from camera_paramters.txt)
        self.camera_extrinsics = {
            'c0': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.049},
                'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.707, 'w': 0.707}
            },
            'c1': {
                'position': {'x': 0.0506, 'y': -0.0013, 'z': 0.0117},
                'rotation': {'x': 0.416, 'y': 0.415, 'z': 0.567, 'w': 0.577}
            },
            'c2': {
                'position': {'x': 0.0300, 'y': -0.0025, 'z': -0.0484},
                'rotation': {'x': -0.673, 'y': -0.673, 'z': -0.216, 'w': -0.221}
            },
            'c3': {
                'position': {'x': -0.0332, 'y': -0.0009, 'z': -0.0465},
                'rotation': {'x': -0.672, 'y': -0.673, 'z': 0.220, 'w': 0.219}
            },
            'c4': {
                'position': {'x': -0.0515, 'y': -0.0003, 'z': 0.0131},
                'rotation': {'x': -0.412, 'y': -0.416, 'z': 0.566, 'w': 0.580}
            },
        }

        # Publish camera parameters once (constant)
        self.publish_camera_parameters()

        # Create timer for periodic image capture and publishing
        self.timer = self.create_timer(0.1, self.capture_and_publish)  # 10 Hz

        self.get_logger().info("Spot Pano Publisher initialized")
        self.frame_count = 0
        self.start_time = time.time()

    def publish_camera_parameters(self):
        """Publish camera intrinsics and extrinsics for all 5 cameras (c0-c4)"""
        for cam_name in ['c0', 'c1', 'c2', 'c3', 'c4']:
            cam_info = CameraInfo()
            cam_info.header = Header()
            cam_info.header.stamp = self.get_clock().now().to_msg()
            cam_info.header.frame_id = cam_name

            intrinsics = self.camera_intrinsics[cam_name]
            cam_info.width = self.camera_width  # 576 with resize_ratio=0.30
            cam_info.height = self.camera_height  # 324 with resize_ratio=0.30
            cam_info.distortion_model = "plumb_bob"

            # K matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            cam_info.k = [
                float(intrinsics['fx']), 0.0, float(intrinsics['cx']),
                0.0, float(intrinsics['fy']), float(intrinsics['cy']),
                0.0, 0.0, 1.0
            ]

            # D distortion coefficients [k1, k2, p1, p2, k3]
            cam_info.d = [
                float(intrinsics['k1']), float(intrinsics['k2']),
                float(intrinsics['p1']), float(intrinsics['p2']), 0.0
            ]

            # P projection matrix (for rectified image)
            cam_info.p = [
                float(intrinsics['fx']), 0.0, float(intrinsics['cx']), 0.0,
                0.0, float(intrinsics['fy']), float(intrinsics['cy']), 0.0,
                0.0, 0.0, 1.0, 0.0
            ]

            # R rectification matrix (identity if no rectification)
            cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

            self.camera_params_pub.publish(cam_info)
            self.get_logger().info(f"Published camera parameters for {cam_name}")

    def capture_and_publish(self):
        """Capture pano image, process, and publish"""
        try:
            # Build request for pano image (using configurable parameters)
            request = [
                build_image_request(
                    'pano',
                    quality_percent=self.quality_percent,
                    resize_ratio=self.resize_ratio,
                    image_format=self.image_format,
                    pixel_format=self.pixel_format
                )
            ]

            # Capture image
            image_responses = self.cam_image_client.get_image(request)
            response = image_responses[0]
            img = response.shot.image

            # Convert to cv2 image
            if img.format == 2:  # RAW
                pano_image = cv2.cvtColor(
                    np.frombuffer(img.data, dtype=np.uint8).reshape((img.rows, img.cols, 3)),
                    cv2.COLOR_RGB2BGR
                )
            else:  # JPEG
                pano_image = cv2.imdecode(
                    np.frombuffer(img.data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )

            # Process: split into 5 fisheyes, rotate, calibrate, stitch
            stitched_pano = self.process_pano(pano_image)

            # Publish stitched panoramic image
            ros_image = self.bridge.cv2_to_imgmsg(stitched_pano, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "pano_stitched"
            self.pano_pub.publish(ros_image)

            # Publish pano parameters (timestamp and extrinsics)
            self.publish_pano_parameters(response)

            self.frame_count += 1

            # Log FPS every 30 frames
            if self.frame_count % 30 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                self.get_logger().info(f"Frame {self.frame_count}: {fps:.2f} FPS")

        except Exception as e:
            self.get_logger().error(f"Error in capture_and_publish: {e}")

    def process_pano(self, pano_image):
        """
        Split pano into 5 fisheyes, rotate 90° clockwise, calibrate, and stitch

        Original 'pano' is 9600x1080 (W x H), each camera is 1920x1080
        With resize_ratio=0.30: pano is 2880x324 (W x H), each camera is 576x324 (W x H)
        In OpenCV format: height=324, width=2880 for pano; height=324, width=576 per camera
        """
        height, width = pano_image.shape[:2]
        camera_width = width // 5  # 576 pixels per camera (W)

        fisheyes = []

        for i in range(5):
            # Extract each fisheye camera region
            x_start = i * camera_width
            x_end = (i + 1) * camera_width
            fisheye = pano_image[:, x_start:x_end].copy()

            # Rotate 90° clockwise
            fisheye_rotated = cv2.rotate(fisheye, cv2.ROTATE_90_CLOCKWISE)

            # Apply calibration (undistort using camera intrinsics)
            cam_name = f'c{i}'
            intrinsics = self.camera_intrinsics[cam_name]

            # Camera matrix
            camera_matrix = np.array([
                [intrinsics['fx'], 0, intrinsics['cx']],
                [0, intrinsics['fy'], intrinsics['cy']],
                [0, 0, 1]
            ], dtype=np.float32)

            # Distortion coefficients
            dist_coeffs = np.array([
                intrinsics['k1'], intrinsics['k2'],
                intrinsics['p1'], intrinsics['p2'], 0
            ], dtype=np.float32)

            # Undistort
            h, w = fisheye_rotated.shape[:2]
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, (w, h), 1, (w, h)
            )

            fisheye_calibrated = cv2.undistort(
                fisheye_rotated, camera_matrix, dist_coeffs,
                None, new_camera_matrix
            )

            # Crop to ROI
            x, y, w, h = roi
            if w > 0 and h > 0:
                fisheye_calibrated = fisheye_calibrated[y:y+h, x:x+w]

            fisheyes.append(fisheye_calibrated)

        # Stitch fisheyes together horizontally
        # Resize all to same height if needed
        max_height = max(f.shape[0] for f in fisheyes)
        fisheyes_resized = []
        for f in fisheyes:
            if f.shape[0] != max_height:
                aspect = f.shape[1] / f.shape[0]
                new_width = int(max_height * aspect)
                f = cv2.resize(f, (new_width, max_height))
            fisheyes_resized.append(f)

        # Horizontal concatenation
        stitched = np.hstack(fisheyes_resized)

        return stitched

    def publish_pano_parameters(self, response):
        """Publish timestamp and extrinsics from image response"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "body"
        transform.child_frame_id = "pano"

        # Extract transform from response
        snapshot = response.shot.transforms_snapshot

        # Find pano transform
        if 'pano' in snapshot.child_to_parent_edge_map:
            pano_transform = snapshot.child_to_parent_edge_map['pano'].parent_tform_child

            # Position
            transform.transform.translation.x = pano_transform.position.x
            transform.transform.translation.y = pano_transform.position.y
            transform.transform.translation.z = pano_transform.position.z

            # Rotation (quaternion)
            transform.transform.rotation.x = pano_transform.rotation.x
            transform.transform.rotation.y = pano_transform.rotation.y
            transform.transform.rotation.z = pano_transform.rotation.z
            transform.transform.rotation.w = pano_transform.rotation.w

        self.pano_params_pub.publish(transform)


def main(args=None):
    rclpy.init(args=args)

    try:
        # Create publisher with adjustable parameters
        # Adjust these values as needed:
        #   resize_ratio: 0.1-1.0 (lower = faster but lower quality)
        #   quality_percent: 1-100 (only affects JPEG format)
        #   image_format: 1=JPEG, 2=RAW
        #   pixel_format: 3=RGB, 4=RGBA
        node = SpotPanoPublisher(
            resize_ratio=0.30,
            quality_percent=100,
            image_format=2,  # RAW
            pixel_format=3   # RGB
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
