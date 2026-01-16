#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge, CvBridgeError
import threading

class DetectionVisualizationCombiner:
    def __init__(self):
        rospy.loginfo("Initializing Detection Visualization Combiner...")
        
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        # Store latest data
        self.original_image = None
        self.yolov8_detections = None
        self.latest_timestamp = None
        
        # Color scheme for different detectors
        self.yolov8_color = (0, 255, 0)      # Green for YOLOv8
        # self.yolov7_color = (255, 0, 255)    # Magenta for YOLOv7+SAM2
        
        # # Subscribers
        # self.image_sub = rospy.Subscriber('/spot_image', Image, 
        #                                  self.image_callback, queue_size=1)
        self.yolov8_sub = rospy.Subscriber('/yolo/detection', Detection2DArray, 
                                          self.yolov8_callback, queue_size=1)
        # self.yolov7_sub = rospy.Subscriber('/yolov7_sam2/detection', Detection2DArray, 
        #                                   self.yolov7_callback, queue_size=1)
        
        # Publisher for combined visualization
        self.combined_pub = rospy.Publisher('/yolo/combined_visualization', Image, queue_size=10)
        


    def image_callback(self, msg):
        """Handle original image messages"""
        try:
            with self.lock:
                self.original_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.latest_timestamp = msg.header.stamp
                self._try_combine_and_publish()
        except CvBridgeError as e:
            rospy.logerr(f"Original image conversion error: {e}")

    def yolov8_callback(self, msg):
        """Handle YOLOv8 detection messages"""
        with self.lock:
            self.yolov8_detections = msg
            self._try_combine_and_publish()

    def _try_combine_and_publish(self):
        """Create combined visualization if original image is available"""
        if self.original_image is not None:
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
        """Create overlay visualization with both detectors' results on the original image"""
        # Start with the original image
        combined = self.original_image.copy()
        
        # Draw YOLOv8 detections in screen
        if self.yolov8_detections is not None:
            combined = self._draw_detections(combined, self.yolov8_detections, 
                                           self.yolov8_color, "YOLOv8")
        
        # Add legend to show active detector
        combined = self._add_legend(combined)
        
        return combined

    def _add_legend(self, image):
        """Add legend showing which detector is active"""
        h, w = image.shape[:2]
        
        # Legend background
        legend_h = 50
        legend_w = 200
        start_x = w - legend_w - 10
        start_y = 10
        
        # Semi-transparent background
        overlay = image.copy()
        cv2.rectangle(overlay, (start_x, start_y), 
                     (start_x + legend_w, start_y + legend_h), 
                     (0, 0, 0), -1)
        image = cv2.addWeighted(image, 0.7, overlay, 0.3, 0)
        
        # Legend text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        
        return image

    def _draw_detections(self, image, detections, color, detector_name):
        """Draw detection results on the image"""
        for detection in detections.detections:
            # Get bounding box
            bbox = detection.bbox
            x1 = int(bbox.center.x - bbox.size_x / 2)
            y1 = int(bbox.center.y - bbox.size_y / 2)
            x2 = int(bbox.center.x + bbox.size_x / 2)
            y2 = int(bbox.center.y + bbox.size_y / 2)
            
            # Draw bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            
            # Get class name and confidence
            if detection.results:
                # Access id and score directly from the results object
                class_name = detection.results[0].id
                confidence = detection.results[0].score
                label = f"{detector_name}: {class_name} {confidence:.2f}"
            else:
                label = f"{detector_name}: Unknown"
            
            # Draw label background and text
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(image, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), color, -1)
            
            # Draw label text
            cv2.putText(image, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            
            # Draw mask if available
            if hasattr(detection, 'source_img') and detection.source_img is not None and detection.source_img.data:
                try:
                    # Convert mask data from source_img to numpy array
                    mask_image = self.bridge.imgmsg_to_cv2(detection.source_img, "mono8")

                    # --- FIX: Add a check and resize the mask to match the image dimensions ---
                    if mask_image.shape != image.shape[:2]:
                        mask_image = cv2.resize(mask_image, (image.shape[1], image.shape[0]), 
                                                interpolation=cv2.INTER_NEAREST)
                    
                    # Create colored mask overlay - only where mask is active
                    mask_indices = mask_image > 0
                    if np.any(mask_indices):
                        # Create overlay only for masked regions to preserve image brightness
                        overlay = image.copy()
                        overlay[mask_indices] = color
                        
                        # Blend only the masked regions with lower alpha to preserve brightness
                        mask_alpha = 0.25  # Reduced alpha to prevent dimming
                        image[mask_indices] = cv2.addWeighted(
                            image[mask_indices], 1 - mask_alpha, 
                            overlay[mask_indices], mask_alpha, 0
                        )
                    
                except Exception as e:
                    rospy.logwarn(f"Failed to draw mask for {detector_name}: {e}")
        
        return image


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