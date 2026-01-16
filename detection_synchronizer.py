#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import defaultdict
import threading
import time

class DetectionSynchronizer:
    def __init__(self):
        rospy.loginfo("Initializing Detection Synchronizer...")
        
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        # Buffer to store detections by timestamp
        self.yolov8_buffer = {}
        self.yolov7_sam2_buffer = {}
        self.max_buffer_size = 50
        self.max_time_diff = 0.5  # Maximum time difference to consider frames synchronized
        
        # Subscribers for both detection streams
        self.yolov8_sub = rospy.Subscriber('/yolo/detection', Detection2DArray, 
                                          self.yolov8_callback, queue_size=10)
        
        # Publishers for combined results
        self.combined_pub = rospy.Publisher('/yolo/combined_detection', Detection2DArray, queue_size=10)
        self.visualization_pub = rospy.Publisher('/yolo/combined_visualization', Image, queue_size=10)
        
        # Image subscriber to get original images for visualization
        self.image_sub = rospy.Subscriber('/spot_image', Image, self.image_callback, queue_size=10)
        self.latest_image = None
        
        # Timer to clean old buffers
        rospy.Timer(rospy.Duration(2.0), self.cleanup_buffers)
        
        rospy.loginfo("Detection Synchronizer initialized!")

    def yolov8_callback(self, msg):
        """Handle YOLOv8 detection messages"""
        if msg.header.frame_id == "yolov8_detections":
            with self.lock:
                timestamp = msg.header.stamp.to_sec()
                self.yolov8_buffer[timestamp] = msg
                self._check_for_matches(timestamp)

    def yolov7_sam2_callback(self, msg):
        """Handle YOLOv7+SAM2 detection messages"""
        if msg.header.frame_id == "yolov7_sam2_detections":
            with self.lock:
                timestamp = msg.header.stamp.to_sec()
                self.yolov7_sam2_buffer[timestamp] = msg
                self._check_for_matches(timestamp)

    def image_callback(self, msg):
        """Store latest image for visualization"""
        self.latest_image = msg

    def _check_for_matches(self, timestamp):
        """Check if we have matching detections from both models"""
        # Look for YOLOv7+SAM2 results within time tolerance
        best_match_time = None
        best_time_diff = float('inf')
        
        for yolo7_time in self.yolov7_sam2_buffer.keys():
            time_diff = abs(timestamp - yolo7_time)
            if time_diff < self.max_time_diff and time_diff < best_time_diff:
                best_time_diff = time_diff
                best_match_time = yolo7_time
        
        # If we found a match, combine the results
        if best_match_time is not None and timestamp in self.yolov8_buffer:
            yolov8_msg = self.yolov8_buffer[timestamp]
            yolov7_sam2_msg = self.yolov7_sam2_buffer[best_match_time]
            
            self._combine_and_publish(yolov8_msg, yolov7_sam2_msg)
            
            # Remove used messages
            del self.yolov8_buffer[timestamp]
            del self.yolov7_sam2_buffer[best_match_time]

    def _combine_and_publish(self, yolov8_msg, yolov7_sam2_msg):
        """Combine detections from both models and publish"""
        combined_msg = Detection2DArray()
        combined_msg.header.stamp = yolov8_msg.header.stamp
        combined_msg.header.frame_id = "combined_detections"
        
        # Add YOLOv8 detections (mark them)
        for detection in yolov8_msg.detections:
            # Modify the detection to indicate it's from YOLOv8
            detection_copy = Detection2D()
            detection_copy.bbox = detection.bbox
            detection_copy.results = detection.results
            detection_copy.source_img = detection.source_img
            # Add a custom field or modify ID to indicate source
            combined_msg.detections.append(detection_copy)
        
        # Add YOLOv7+SAM2 detections (mark them)
        for detection in yolov7_sam2_msg.detections:
            # Modify the detection to indicate it's from YOLOv7+SAM2
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
        """Create a visualization showing both detection results"""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8").copy()
            
            # Draw YOLOv8 detections in blue
            for detection in yolov8_msg.detections:
                bbox = detection.bbox
                x1 = int(bbox.center.x - bbox.size_x / 2)
                y1 = int(bbox.center.y - bbox.size_y / 2)
                x2 = int(bbox.center.x + bbox.size_x / 2)
                y2 = int(bbox.center.y + bbox.size_y / 2)
                
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue for YOLOv8
                if detection.results:
                    label = f"YOLOv8: {detection.results[0].score:.2f}"
                    cv2.putText(cv_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
            # Draw YOLOv7+SAM2 detections in red
            for detection in yolov7_sam2_msg.detections:
                bbox = detection.bbox
                x1 = int(bbox.center.x - bbox.size_x / 2)
                y1 = int(bbox.center.y - bbox.size_y / 2)
                x2 = int(bbox.center.x + bbox.size_x / 2)
                y2 = int(bbox.center.y + bbox.size_y / 2)
                
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red for YOLOv7+SAM2
                if detection.results:
                    label = f"YOLOv7+SAM2: {detection.results[0].score:.2f}"
                    cv2.putText(cv_img, label, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Add legend
            cv2.putText(cv_img, "Blue: YOLOv8", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(cv_img, "Red: YOLOv7+SAM2", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            self.visualization_pub.publish(vis_msg)
            
        except Exception as e:
            rospy.logerr(f"Failed to create combined visualization: {e}")

    def cleanup_buffers(self, event):
        """Remove old entries from buffers"""
        current_time = time.time()
        with self.lock:
            # Clean YOLOv8 buffer
            old_keys = [k for k in self.yolov8_buffer.keys() if current_time - k > 5.0]
            for key in old_keys:
                del self.yolov8_buffer[key]
            
            # Clean YOLOv7+SAM2 buffer
            old_keys = [k for k in self.yolov7_sam2_buffer.keys() if current_time - k > 5.0]
            for key in old_keys:
                del self.yolov7_sam2_buffer[key]
            
            # Limit buffer size
            if len(self.yolov8_buffer) > self.max_buffer_size:
                oldest_keys = sorted(self.yolov8_buffer.keys())[:-self.max_buffer_size]
                for key in oldest_keys:
                    del self.yolov8_buffer[key]
            
            if len(self.yolov7_sam2_buffer) > self.max_buffer_size:
                oldest_keys = sorted(self.yolov7_sam2_buffer.keys())[:-self.max_buffer_size]
                for key in oldest_keys:
                    del self.yolov7_sam2_buffer[key]

def main():
    rospy.init_node('detection_synchronizer', anonymous=True)
    
    try:
        synchronizer = DetectionSynchronizer()
        
        # Subscribe to YOLOv7+SAM2 after a delay to ensure it's started
        rospy.Timer(rospy.Duration(2.0), lambda _: setattr(synchronizer, 'yolov7_sam2_sub', 
                   rospy.Subscriber('/yolo/detection', Detection2DArray, 
                                  synchronizer.yolov7_sam2_callback, queue_size=10)), oneshot=True)
        
        rospy.loginfo("Detection Synchronizer started successfully!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Detection Synchronizer interrupted")
    except Exception as e:
        rospy.logerr(f"Failed to start Detection Synchronizer: {e}")
    finally:
        rospy.loginfo("Detection Synchronizer shutting down")

if __name__ == '__main__':
    main()
