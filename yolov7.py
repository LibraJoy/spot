#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
import time
import PIL.Image
import matplotlib.pyplot as plt
import os
import threading
from collections import defaultdict

# ROS imports
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header

# SAM2 imports
import sam2
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

# Note: YOLOv7 will be loaded via torch.hub, no direct import needed


class YOLOv7SAM2CombinedDetector:
    def __init__(self):
        rospy.loginfo("Initializing YOLOv7+SAM2 Combined Detector...")
        
        self.w_org = 1280
        self.h_org = 720
        self.bridge = CvBridge()
        self.id = 0
        self.img_data = None
        
        # Synchronizer variables
        self.lock = threading.Lock()
        self.yolov8_buffer = {}
        self.max_buffer_size = 50
        self.max_time_diff = 0.5  # Maximum time difference to consider frames synchronized
        self.latest_image = None
        
        # Setup device
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
            self._setup_cuda_optimizations()
        elif torch.backends.mps.is_available():
            self.device = torch.device("mps")
            rospy.logwarn("MPS device support is preliminary")
        else:
            self.device = torch.device("cpu")
            rospy.logwarn("Using CPU device - performance may be limited")
        
        rospy.loginfo(f"YOLOv7+SAM2 using device: {self.device}")
        
        # Load models
        self._load_yolov7_model()
        self._load_sam2_model()
        
        # ROS subscribers and publishers
        self.img_sub = rospy.Subscriber('/spot_image', Image, self.image_callback, queue_size=1)
        
        # Subscribe to YOLOv8 detections for synchronization
        self.yolov8_sub = rospy.Subscriber('/yolo/detection', Detection2DArray, 
                                          self.yolov8_callback, queue_size=10)
        
        # Publishers
        self.yolo_detect_pub = rospy.Publisher('/yolo/detection', Detection2DArray, queue_size=10)
        self.yolo_vis_pub = rospy.Publisher('/yolov7_sam2/visualization', Image, queue_size=10)
        self.yolo_mask_pub = rospy.Publisher('/yolov7_sam2/mask', Image, queue_size=10)
        
        # Combined result publishers
        self.combined_pub = rospy.Publisher('/yolo/combined_detection', Detection2DArray, queue_size=10)
        self.visualization_pub = rospy.Publisher('/yolo/combined_visualization', Image, queue_size=10)
        
        # Timer to clean old buffers
        rospy.Timer(rospy.Duration(2.0), self.cleanup_buffers)
        
        rospy.loginfo("YOLOv7+SAM2 Combined Detector initialized successfully!")

    def _setup_cuda_optimizations(self):
        """Setup CUDA optimizations"""
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
        if torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

    def _load_yolov7_model(self):
        """Load YOLOv7 model"""
        try:
            # Try multiple possible paths for the model (in order of preference)
            model_paths = [
                "/root/spot_ws/src/spot/models/best.7.10.pt",  # Docker path
                "/home/cerlab/ros_noetic_docker_spot/spot_ws/src/spot/models/best.7.10.pt",  # Host path
                "/home/cerlab/Downloads/best.7.10.pt",  # Downloads backup
                "/home/cerlab/ros_noetic_docker_home/sam2_ws/yolo/best.7.10.pt",  # Alternative location
                "best.7.10.pt"  # Current directory
            ]
            
            model_loaded = False
            for path in model_paths:
                try:
                    rospy.loginfo(f"Attempting to load YOLOv7 model from: {path}")
                    
                    # Correct way to load YOLOv7 model using torch.hub
                    self.model = torch.hub.load('WongKinYiu/yolov7', 'custom', path, trust_repo=True)
                    
                    if self.device.type != "cpu":
                        self.model = self.model.to(self.device)
                    
                    rospy.loginfo(f"✓ YOLOv7 model successfully loaded from: {path}")
                    model_loaded = True
                    break
                    
                except Exception as e:
                    rospy.logwarn(f"✗ Failed to load model from {path}: {e}")
                    continue
            
            if not model_loaded:
                raise Exception("Could not load YOLOv7 model from any of the attempted paths")
                
        except Exception as e:
            rospy.logerr(f"Failed to load YOLOv7 model: {e}")
            raise

    def _load_sam2_model(self):
        """Load SAM2 model"""
        try:
            root_path = os.path.dirname(sam2.__file__)
            root_path = os.path.dirname(root_path)
            sam2_checkpoint = f"{root_path}/checkpoints/sam2.1_hiera_tiny.pt"
            model_cfg = "configs/sam2.1/sam2.1_hiera_t.yaml"
            
            sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=self.device)
            self.sam2_predictor = SAM2ImagePredictor(sam2_model)
            rospy.loginfo("SAM2 model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load SAM2 model: {e}")
            raise

    def yolov8_callback(self, msg):
        """Handle YOLOv8 detection messages for synchronization"""
        if msg.header.frame_id == "yolov8_detections":
            with self.lock:
                timestamp = msg.header.stamp.to_sec()
                self.yolov8_buffer[timestamp] = msg
                self._check_for_matches(timestamp)

    def _check_for_matches(self, yolov8_timestamp):
        """Check if we have matching YOLOv7+SAM2 detections for synchronization"""
        # Only proceed if we have recent YOLOv7+SAM2 detection
        if not (hasattr(self, 'last_yolov7_detection') and hasattr(self, 'last_yolov7_timestamp')):
            return
            
        time_diff = abs(yolov8_timestamp - self.last_yolov7_timestamp)
        
        if time_diff < self.max_time_diff and yolov8_timestamp in self.yolov8_buffer:
            yolov8_msg = self.yolov8_buffer[yolov8_timestamp]
            yolov7_sam2_msg = self.last_yolov7_detection
            
            self._combine_and_publish(yolov8_msg, yolov7_sam2_msg)
            
            # Remove used YOLOv8 message
            del self.yolov8_buffer[yolov8_timestamp]

    def show_mask(self, mask, img, random_color=True):
        """Apply mask overlay to image"""
        if mask is None or img is None:
            rospy.logwarn("Mask or image is None!")
            return img
        
        # Ensure mask is 8-bit binary
        mask = mask.astype(np.uint8) * 255
        mask_image = np.zeros((*mask.shape, 3), dtype=np.uint8)
        
        # Generate color
        color = np.random.rand(3) if random_color else np.array([1, 0, 0])  # Default red
        mask_image[mask > 0] = (color * 255).astype(np.uint8)
        
        # Resize mask_image if dimensions don't match
        if mask_image.shape[:2] != img.shape[:2]:
            mask_image = cv2.resize(mask_image, (img.shape[1], img.shape[0]))
        
        # Ensure both images are 3-channel
        if len(img.shape) == 2 or img.shape[2] == 1:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        if mask_image.shape[2] == 1:
            mask_image = cv2.cvtColor(mask_image, cv2.COLOR_GRAY2BGR)
        
        # Blend mask with image
        alpha = 0.6
        overlay = cv2.addWeighted(img, 1, mask_image, alpha, 0)
        return overlay

    def image_callback(self, data):
        """Main callback for processing incoming images"""
        try:
            # Store latest image for visualization
            self.latest_image = data
            
            # Convert ROS image to OpenCV format
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_data = cv_img.copy()
            
            # Perform YOLOv7 detection
            start_time = time.time()
            results = self.model(cv_img)
            names = self.model.names
            yolo_time = time.time() - start_time
            
            rospy.logdebug(f"YOLO execution time: {yolo_time:.6f} seconds")
            
            # Process and publish results
            self.publish_results(results, names, data)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")

    def publish_results(self, results, names, ros_image_msg):
        """Process YOLOv7 and SAM2 detections and publish results"""
        start_time = time.time()
        self.id += 1
        
        # Extract detections from YOLOv7 results
        p = results.pred[0]
        if len(p) == 0:
            rospy.logdebug("No detections found")
            self._publish_empty_results(ros_image_msg)
            return
        
        box, conf, cat = p[:, :4], p[:, 4], p[:, 5]
        input_boxes = box.int()
        
        # Prepare image for SAM2
        img_overlay = np.array(self.img_data, copy=True)
        image_rgb = cv2.cvtColor(img_overlay, cv2.COLOR_BGR2RGB)
        
        # SAM2 segmentation
        sam_start = time.time()
        self.sam2_predictor.set_image(image_rgb)
        
        masks = None
        mask_base = np.zeros(img_overlay.shape[:2], dtype=np.uint8)
        
        if input_boxes.size(0) > 0:
            masks, scores, _ = self.sam2_predictor.predict(
                point_coords=None,
                point_labels=None,
                box=input_boxes,
                multimask_output=False,
            )
            
            # Apply masks to create overlay and combined mask
            for mask in masks:
                mask = mask.squeeze(0) if mask.shape[0] == 1 else mask
                mask_base = np.where(mask_base + mask > 0, 1, 0)
                img_overlay = self.show_mask(mask, img_overlay)
        
        sam_time = time.time() - sam_start
        
        # Draw bounding boxes and labels
        self._draw_bounding_boxes(img_overlay, input_boxes, conf, cat, names)
        
        # Publish all results
        yolov7_detection_msg = self._publish_detection_results(
            ros_image_msg, box, conf, cat, names, masks, mask_base, img_overlay
        )
        
        # Store for synchronization
        with self.lock:
            self.last_yolov7_detection = yolov7_detection_msg
            self.last_yolov7_timestamp = ros_image_msg.header.stamp.to_sec()
        
        total_time = time.time() - start_time
        rospy.logdebug(f"Total processing time: {total_time:.6f} seconds (SAM2: {sam_time:.6f}s)")

    def _draw_bounding_boxes(self, img, boxes, confs, cats, names):
        """Draw bounding boxes and labels on image"""
        tl = 1  # line thickness
        
        for (x1, y1, x2, y2), conf_score, category in zip(boxes, confs, cats):
            label = f"{names[int(category)]} {conf_score * 100:.1f}%"
            color = (0, 0, 255)  # Red
            
            c1, c2 = (int(x1), int(y1)), (int(x2), int(y2))
            cv2.rectangle(img, c1, c2, color, tl, lineType=cv2.LINE_AA)
            
            # Add label
            tf = max(tl - 1, 1)
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2_label = (c1[0] + t_size[0], c1[1] - t_size[1] - 3)
            cv2.rectangle(img, c1, c2_label, color, -1, cv2.LINE_AA)
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, (225, 255, 255), thickness=tf, lineType=cv2.LINE_AA)

    def _publish_detection_results(self, ros_image_msg, boxes, confs, cats, names, masks, mask_base, img_overlay):
        """Publish detection results to all topics"""
        try:
            # Convert images to ROS messages
            vis_msg = self.bridge.cv2_to_imgmsg(img_overlay, encoding="bgr8")
            mask_base_8bit = (mask_base * 255).astype(np.uint8)
            mask_msg = self.bridge.cv2_to_imgmsg(mask_base_8bit, encoding="mono8")
            
            # Create Detection2DArray for shared /yolo/detection topic
            detection_array = Detection2DArray()
            detection_array.header.stamp = ros_image_msg.header.stamp
            detection_array.header.frame_id = "yolov7_sam2_detections"  # Distinguish YOLOv7+SAM2 results
            
            for i, (box, conf, cat) in enumerate(zip(boxes, confs, cats)):
                # Create Detection2D
                detection = Detection2D()
                detection.bbox.center.x = float((box[0] + box[2]) / 2)
                detection.bbox.center.y = float((box[1] + box[3]) / 2)
                detection.bbox.size_x = float(abs(box[2] - box[0]))
                detection.bbox.size_y = float(abs(box[3] - box[1]))
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = int(cat)
                hypothesis.score = float(conf)
                detection.results.append(hypothesis)
                
                # Add individual mask if available
                if masks is not None and i < len(masks):
                    try:
                        mask = masks[i]
                        
                        # Handle different mask shapes properly
                        if mask.ndim == 3:
                            if mask.shape[0] == 1:
                                mask_img = mask[0]
                            elif mask.shape[2] == 1:
                                mask_img = mask[:, :, 0]
                            else:
                                mask_img = mask
                        elif mask.ndim == 2:
                            mask_img = mask
                        else:
                            rospy.logwarn(f"Unexpected mask dimensions: {mask.shape}")
                            continue
                        
                        # Ensure mask is binary and convert to uint8
                        mask_img = (mask_img > 0.5).astype(np.uint8) * 255
                        mask_ros_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding="mono8")
                        detection.source_img = mask_ros_msg
                        
                    except Exception as e:
                        rospy.logwarn(f"Failed to attach mask to detection {i}: {e}")
                
                detection_array.detections.append(detection)
            
            # Publish all messages
            self.yolo_vis_pub.publish(vis_msg)
            self.yolo_mask_pub.publish(mask_msg)
            self.yolo_detect_pub.publish(detection_array)  # Shared topic with YOLOv8
            
            rospy.loginfo(f"Published YOLOv7+SAM2 detection #{self.id} with {len(detection_array.detections)} detections")
            
            return detection_array
            
        except Exception as e:
            rospy.logerr(f"Failed to publish results: {e}")
            return None

    def _publish_empty_results(self, ros_image_msg):
        """Publish empty results when no detections are found"""
        try:
            # Create empty detection array
            detection_array = Detection2DArray()
            detection_array.header.stamp = ros_image_msg.header.stamp
            detection_array.header.frame_id = "yolov7_sam2_detections"
            
            # Publish empty results to shared topic
            self.yolo_detect_pub.publish(detection_array)
            
            # Store empty detection for synchronization
            with self.lock:
                self.last_yolov7_detection = detection_array
                self.last_yolov7_timestamp = ros_image_msg.header.stamp.to_sec()
            
        except Exception as e:
            rospy.logerr(f"Failed to publish empty results: {e}")

    def _combine_and_publish(self, yolov8_msg, yolov7_sam2_msg):
        """Combine detections from both models and publish"""
        combined_msg = Detection2DArray()
        combined_msg.header.stamp = yolov8_msg.header.stamp
        combined_msg.header.frame_id = "combined_detections"
        
        # Add YOLOv8 detections
        for detection in yolov8_msg.detections:
            detection_copy = Detection2D()
            detection_copy.bbox = detection.bbox
            detection_copy.results = detection.results
            detection_copy.source_img = detection.source_img
            combined_msg.detections.append(detection_copy)
        
        # Add YOLOv7+SAM2 detections
        for detection in yolov7_sam2_msg.detections:
            detection_copy = Detection2D()
            detection_copy.bbox = detection.bbox
            detection_copy.results = detection.results
            detection_copy.source_img = detection.source_img
            combined_msg.detections.append(detection_copy)
        
        # Publish combined results
        self.combined_pub.publish(combined_msg)
        
        # Create visualization if we have an image
        if self.latest_image is not None:
            self._create_combined_visualization(combined_msg, yolov8_msg, yolov7_sam2_msg)
        
        rospy.loginfo(f"Combined detection published: {len(yolov8_msg.detections)} YOLOv8 + {len(yolov7_sam2_msg.detections)} YOLOv7+SAM2 detections")

    def _create_combined_visualization(self, combined_msg, yolov8_msg, yolov7_sam2_msg):
        """Create a visualization showing both detection results with masks"""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8").copy()
            
            # Apply masks with different colors
            # YOLOv7+SAM2 masks (red)
            for detection in yolov7_sam2_msg.detections:
                if detection.source_img and detection.source_img.data:
                    try:
                        mask_img = self.bridge.imgmsg_to_cv2(detection.source_img, "mono8")
                        mask_colored = np.zeros_like(cv_img)
                        mask_colored[:, :, 2] = mask_img  # Red channel
                        cv_img = cv2.addWeighted(cv_img, 1, mask_colored, 0.3, 0)
                    except Exception as e:
                        rospy.logdebug(f"Failed to apply YOLOv7+SAM2 mask: {e}")
            
            # YOLOv8 masks (blue)
            for detection in yolov8_msg.detections:
                if detection.source_img and detection.source_img.data:
                    try:
                        mask_img = self.bridge.imgmsg_to_cv2(detection.source_img, "mono8")
                        mask_colored = np.zeros_like(cv_img)
                        mask_colored[:, :, 0] = mask_img  # Blue channel
                        cv_img = cv2.addWeighted(cv_img, 1, mask_colored, 0.3, 0)
                    except Exception as e:
                        rospy.logdebug(f"Failed to apply YOLOv8 mask: {e}")
            
            # Draw bounding boxes
            # YOLOv8 (blue boxes)
            for detection in yolov8_msg.detections:
                bbox = detection.bbox
                x1, y1 = int(bbox.center.x - bbox.size_x / 2), int(bbox.center.y - bbox.size_y / 2)
                x2, y2 = int(bbox.center.x + bbox.size_x / 2), int(bbox.center.y + bbox.size_y / 2)
                
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                if detection.results:
                    label = f"YOLOv8: {detection.results[0].score:.2f}"
                    cv2.putText(cv_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
            # YOLOv7+SAM2 (red boxes)
            for detection in yolov7_sam2_msg.detections:
                bbox = detection.bbox
                x1, y1 = int(bbox.center.x - bbox.size_x / 2), int(bbox.center.y - bbox.size_y / 2)
                x2, y2 = int(bbox.center.x + bbox.size_x / 2), int(bbox.center.y + bbox.size_y / 2)
                
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                if detection.results:
                    label = f"YOLOv7+SAM2: {detection.results[0].score:.2f}"
                    cv2.putText(cv_img, label, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            self.visualization_pub.publish(vis_msg)
            
        except Exception as e:
            rospy.logerr(f"Failed to create combined visualization: {e}")

    def cleanup_buffers(self, event):
        """Remove old entries from buffers"""
        current_time = time.time()
        with self.lock:
            # Clean YOLOv8 buffer (remove entries older than 5 seconds)
            old_timestamps = [t for t in self.yolov8_buffer.keys() if current_time - t > 5.0]
            for timestamp in old_timestamps:
                del self.yolov8_buffer[timestamp]
            
            # Limit buffer size to prevent memory overflow
            if len(self.yolov8_buffer) > self.max_buffer_size:
                oldest_timestamps = sorted(self.yolov8_buffer.keys())[:-self.max_buffer_size]
                for timestamp in oldest_timestamps:
                    del self.yolov8_buffer[timestamp]
                    
            rospy.logdebug(f"Buffer cleanup: {len(self.yolov8_buffer)} YOLOv8 messages remaining")


def main():
    rospy.init_node('yolov7_sam2_combined_detector', anonymous=True)
    
    try:
        detector = YOLOv7SAM2CombinedDetector()
        rospy.loginfo("YOLOv7+SAM2 Combined Detector started successfully!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLOv7+SAM2 Combined Detector interrupted")
    except Exception as e:
        rospy.logerr(f"Failed to start YOLOv7+SAM2 Combined Detector: {e}")
    finally:
        rospy.loginfo("YOLOv7+SAM2 Combined Detector shutting down")


if __name__ == '__main__':
    main()
