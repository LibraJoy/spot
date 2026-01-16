#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
import time
import PIL.Image
import matplotlib.pyplot as plt
import os

# ROS imports
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header

# SAM2 imports
import sam2
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

import yolov7


class YOLOv7SAM2ImmediateDetector:
    def __init__(self):
        rospy.loginfo("Initializing YOLOv7+SAM2 Immediate Detector (No Synchronization)...")
        
        self.w_org = 1280
        self.h_org = 720
        self.bridge = CvBridge()
        self.id = 0
        self.img_data = None
        
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
        
        # Publishers - IMMEDIATE publication, no synchronization
        self.yolo_detect_pub = rospy.Publisher('/yolo/detection', Detection2DArray, queue_size=10)
        self.yolo_vis_pub = rospy.Publisher('/yolov7_sam2/visualization', Image, queue_size=10)
        self.yolo_mask_pub = rospy.Publisher('/yolov7_sam2/mask', Image, queue_size=10)
        
        rospy.loginfo("YOLOv7+SAM2 Immediate Detector initialized successfully!")
        rospy.loginfo("Publishing detections IMMEDIATELY - no synchronization delays!")

    def _setup_cuda_optimizations(self):
        """Setup CUDA optimizations"""
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
        if torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

    def _load_yolov7_model(self):
        """Load YOLOv7 model exactly like the working yolo_sam2.py"""
        try:
            self.model_path = "/root/spot_ws/src/spot/models/best.7.10.pt"
            
            rospy.loginfo(f"Loading YOLOv7 model from: {self.model_path}")
            
            # Use exactly the same method as your working yolo_sam2.py
            if torch.cuda.is_available():
                self.model = yolov7.load(self.model_path).to(self.device)
            elif torch.backends.mps.is_available():
                self.model = yolov7.load(self.model_path)
            else:
                self.model = yolov7.load(self.model_path)
            
            # rospy.loginfo(f"✓ YOLOv7 model successfully loaded")
                
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
            # rospy.loginfo("SAM2 model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load SAM2 model: {e}")
            raise

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
            self._publish_detection_results(results, names, data)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")

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

    def _publish_detection_results(self, results, names, ros_image_msg):
        """Process YOLOv7 and SAM2 detections and publish results immediately"""
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
        
        try:
            # Convert images to ROS messages
            vis_msg = self.bridge.cv2_to_imgmsg(img_overlay, encoding="bgr8")
            mask_base_8bit = (mask_base * 255).astype(np.uint8)
            mask_msg = self.bridge.cv2_to_imgmsg(mask_base_8bit, encoding="mono8")
            
            # Create Detection2DArray for shared /yolo/detection topic
            detection_array = Detection2DArray()
            detection_array.header.stamp = ros_image_msg.header.stamp
            detection_array.header.frame_id = "yolov7_sam2_detections"  # Distinguish YOLOv7+SAM2 results
            
            for i, (bbox, conf_score, cat_id) in enumerate(zip(box, conf, cat)):
                # Create Detection2D
                detection = Detection2D()
                detection.bbox.center.x = float((bbox[0] + bbox[2]) / 2)
                detection.bbox.center.y = float((bbox[1] + bbox[3]) / 2)
                detection.bbox.size_x = float(abs(bbox[2] - bbox[0]))
                detection.bbox.size_y = float(abs(bbox[3] - bbox[1]))
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = int(cat_id)
                hypothesis.score = float(conf_score)
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
            
            # Publish all messages immediately
            self.yolo_vis_pub.publish(vis_msg)
            self.yolo_mask_pub.publish(mask_msg)
            self.yolo_detect_pub.publish(detection_array)  # Shared topic with YOLOv8
            
            total_time = time.time() - start_time
            # rospy.loginfo(f"Published YOLOv7+SAM2 detection #{self.id} with {len(detection_array.detections)} detections")
            # rospy.logdebug(f"Total processing time: {total_time:.6f} seconds (SAM2: {sam_time:.6f}s)")
            
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
            
        except Exception as e:
            rospy.logerr(f"Failed to publish empty results: {e}")


def main():
    rospy.init_node('yolov7_sam2_immediate_detector', anonymous=True)
    
    try:
        detector = YOLOv7SAM2ImmediateDetector()
        rospy.loginfo("YOLOv7+SAM2 Immediate Detector started successfully!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLOv7+SAM2 Immediate Detector interrupted")
    except Exception as e:
        rospy.logerr(f"Failed to start YOLOv7+SAM2 Immediate Detector: {e}")
    finally:
        rospy.loginfo("YOLOv7+SAM2 Immediate Detector shutting down")


if __name__ == '__main__':
    main()
