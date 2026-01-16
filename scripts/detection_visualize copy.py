#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading

class DetectionVisualizationCombiner:
    def __init__(self):
        rospy.loginfo("Initializing Detection Visualization Combiner...")
        
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        # Store latest images from both detectors
        self.yolov8_image = None
        self.yolov7_image = None
        self.latest_timestamp = None
        
        # Subscribers for both visualization streams
        self.yolov8_sub = rospy.Subscriber('/yolo/visualization', Image, 
                                          self.yolov8_callback, queue_size=1)
        self.yolov7_sub = rospy.Subscriber('/yolov7_sam2/visualization', Image, 
                                          self.yolov7_callback, queue_size=1)
        
        # Publisher for combined visualization
        self.combined_pub = rospy.Publisher('/yolo/combined_visualization', Image, queue_size=10)
        
        rospy.loginfo("Detection Visualization Combiner initialized!")
        rospy.loginfo("Subscribing to:")
        rospy.loginfo("  - /yolo/visualization (YOLOv8)")
        rospy.loginfo("  - /yolov7_sam2/visualization (YOLOv7+SAM2)")
        rospy.loginfo("Publishing to:")
        rospy.loginfo("  - /yolo/combined_visualization")

    def yolov8_callback(self, msg):
        """Handle YOLOv8 visualization messages"""
        try:
            with self.lock:
                self.yolov8_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.latest_timestamp = msg.header.stamp
                self._try_combine_and_publish()
        except CvBridgeError as e:
            rospy.logerr(f"YOLOv8 image conversion error: {e}")

    def yolov7_callback(self, msg):
        """Handle YOLOv7+SAM2 visualization messages"""
        try:
            with self.lock:
                self.yolov7_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self._try_combine_and_publish()
        except CvBridgeError as e:
            rospy.logerr(f"YOLOv7+SAM2 image conversion error: {e}")

    def _try_combine_and_publish(self):
        """Combine visualizations if both are available"""
        if self.yolov8_image is not None and self.yolov7_image is not None:
            try:
                combined_image = self._create_combined_visualization()
                
                # Convert to ROS message and publish
                combined_msg = self.bridge.cv2_to_imgmsg(combined_image, encoding="bgr8")
                if self.latest_timestamp:
                    combined_msg.header.stamp = self.latest_timestamp
                else:
                    combined_msg.header.stamp = rospy.Time.now()
                combined_msg.header.frame_id = "combined_detections"
                
                self.combined_pub.publish(combined_msg)
                
            except Exception as e:
                rospy.logerr(f"Failed to create combined visualization: {e}")

    def _create_combined_visualization(self):
        """Create side-by-side or overlay combined visualization"""
        # Ensure both images have the same dimensions
        h1, w1 = self.yolov8_image.shape[:2]
        h2, w2 = self.yolov7_image.shape[:2]
        
        # Resize to match if needed
        if (h1, w1) != (h2, w2):
            target_h, target_w = max(h1, h2), max(w1, w2)
            self.yolov8_image = cv2.resize(self.yolov8_image, (target_w, target_h))
            self.yolov7_image = cv2.resize(self.yolov7_image, (target_w, target_h))
        
        # Method 1: Side-by-side comparison (default)
        combined = self._create_side_by_side()
        
        # Method 2: Overlay (alternative - uncomment to use)
        # combined = self._create_overlay()
        
        return combined

    def _create_side_by_side(self):
        """Create side-by-side visualization"""
        h, w = self.yolov8_image.shape[:2]
        
        # Create combined image
        combined = np.zeros((h, w * 2, 3), dtype=np.uint8)
        
        # Place YOLOv8 on left
        combined[:, :w] = self.yolov8_image
        
        # Place YOLOv7+SAM2 on right  
        combined[:, w:] = self.yolov7_image
        
        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.0
        thickness = 2
        
        # YOLOv8 label (top-left)
        cv2.putText(combined, "YOLOv8", (10, 30), font, font_scale, (0, 255, 255), thickness)
        
        # YOLOv7+SAM2 label (top-right section)
        cv2.putText(combined, "YOLOv7+SAM2", (w + 10, 30), font, font_scale, (0, 100, 255), thickness)
        
        # Add separator line
        cv2.line(combined, (w, 0), (w, h), (255, 255, 255), 2)
        
        return combined

    def _create_overlay(self):
        """Create overlay visualization (alternative method)"""
        # Blend the two images with transparency
        alpha = 0.6  # Weight for YOLOv8 image
        beta = 0.4   # Weight for YOLOv7+SAM2 image
        
        combined = cv2.addWeighted(self.yolov8_image, alpha, self.yolov7_image, beta, 0)
        
        # Add legend
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        
        # Create legend background
        legend_h, legend_w = 60, 300
        legend = np.zeros((legend_h, legend_w, 3), dtype=np.uint8)
        
        # Add legend text
        cv2.putText(legend, "Blue: YOLOv8", (10, 20), font, font_scale, (255, 255, 0), thickness)
        cv2.putText(legend, "Red: YOLOv7+SAM2", (10, 45), font, font_scale, (0, 100, 255), thickness)
        
        # Place legend on combined image
        h, w = combined.shape[:2]
        combined[10:10+legend_h, 10:10+legend_w] = legend
        
        return combined


def main():
    rospy.init_node('detection_visualization_combiner', anonymous=True)
    
    try:
        combiner = DetectionVisualizationCombiner()
        rospy.loginfo("Detection Visualization Combiner started successfully!")
        rospy.loginfo("Now run: rostopic echo /yolo/combined_visualization")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Detection Visualization Combiner interrupted")
    except Exception as e:
        rospy.logerr(f"Failed to start Detection Visualization Combiner: {e}")
    finally:
        rospy.loginfo("Detection Visualization Combiner shutting down")


if __name__ == '__main__':
    main()