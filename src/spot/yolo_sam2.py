#!/usr/bin/env python3
import PIL.Image
import cv2
import random
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import yolov7  # Import YOLOv7
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import PIL.Image
import matplotlib.pyplot as plt
import os
import time
import torch
import random
import numpy as np
import sam2
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
import threading
import copy

from sensor_msgs.msg import Image
import rospy


class SAM2:
    def __init__(self):
        self.w_org = 1280
        self.h_org = 720
        # self.model_path = "/root/spot_ws/src/spot/models/yolov7.pt"
        self.model_path = "/root/spot_ws/src/spot/models/best.7.10.pt"
        # Load YOLOv7 model
        if torch.cuda.is_available():
            device = torch.device("cuda")
            self.model = yolov7.load(self.model_path).to(device)  # YOLOv7 model path
        elif torch.backends.mps.is_available():
            device = torch.device("mps")
            self.model = yolov7.load(self.model_path)  # YOLOv7 model path
        else:
            device = torch.device("cpu")
            self.model = yolov7.load(self.model_path)  # YOLOv7 model path
        print(f"using device: {device}")
        self.device = device
        self.bridge = CvBridge()
        self.sam_infer_complete = True

        if self.device.type == "cuda":
            # use bfloat16 for the entire notebook
            torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
            # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
            if torch.cuda.get_device_properties(0).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True
        elif self.device.type == "mps":
            print(
                "\nSupport for MPS devices is preliminary. SAM 2 is trained with CUDA and might "
                "give numerically different outputs and sometimes degraded performance on MPS. "
                "See e.g. https://github.com/pytorch/pytorch/issues/84936 for a discussion."
            )
        root_path = os.path.dirname(sam2.__file__)
        root_path = os.path.dirname(root_path)
        # print("1")
        sam2_checkpoint = f"{root_path}/checkpoints/sam2.1_hiera_tiny.pt"
        model_cfg = "configs/sam2.1/sam2.1_hiera_t.yaml"

        # print("2")
        sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=self.device)
        self.sam2_predictor = SAM2ImagePredictor(sam2_model)
        self.id=0
        # print("SAM2 model loaded successfully.")
        self.img_sub = rospy.Subscriber('/spot_image', Image, self.image_callback)
        self.yolo_vis_pub = rospy.Publisher('/sam/visualization', Image, queue_size=10)
        self.yolo_detect_pub = rospy.Publisher('/sam/detection', Detection2DArray, queue_size=10)
        
        # Flag to track inference processing
        self.processing = False  
        self.latest_image = None  
        self.lock = threading.Lock()  # Lock for thread-safe access
        
        # Start a separate thread for processing images
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        
    def image_callback(self, msg):
        """Receives the latest image and updates the reference for processing."""
        with self.lock:
            self.latest_image = msg  # Store only the latest image
            # print(f"received image: {msg.header.stamp}")
        # rospy.loginfo("Received new image.")

    def process_images(self):
        """Continuously process the latest image."""
        while not rospy.is_shutdown():
            if self.latest_image and not self.processing:
                with self.lock:
                    img_to_process = copy.deepcopy(self.latest_image)  # Get the latest image
                    self.latest_image = None  # Reset latest image
                self.run_inference(img_to_process)

    def run_inference(self, msg):
        """Runs YOLO and SAM2 inference on the given image."""
        self.processing = True  # Mark as processing
        try:
            if msg is None:
                rospy.loginfo("No image received, skipping processing.")
                return

            timestamp = msg.header.stamp
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').copy()
            if cv_img.shape[0] == 0  or cv_img.shape[1] == 0 or cv_img.shape[2] == 0:
                rospy.loginfo("cv_img shape is 0")
            image_resized = cv2.resize(cv_img, (640, 640))
            
            # Convert BGR to RGB and preprocess for YOLO
            rgb_img = image_resized.transpose((2, 0, 1))[::-1]
            model_img = torch.from_numpy(np.ascontiguousarray(rgb_img)).float() / 255.0
            model_img = model_img.unsqueeze(0).to(self.device)

            # Run YOLO inference
            pred = self.model(model_img)
            p = yolov7.utils.general.non_max_suppression(pred[0], conf_thres=0.8, iou_thres=0.45)

            if not p:
                # rospy.loginfo("No objects detected.")
                self.yolo_vis_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8"))
                return

            p = p[0]
            resized_box = p[:, :4].clone()
            
            # Scale bounding boxes back to original size
            scale_x = cv_img.shape[1] / image_resized.shape[1]
            scale_y = cv_img.shape[0] / image_resized.shape[0]
            box = p[:, :4].clone()
            box[:, [0, 2]] *= scale_x
            box[:, [1, 3]] *= scale_y
            conf, cat = p[:, 4], p[:, 5]

            # Run SAM2 segmentation
            self.sam2_predictor.set_image(cv_img)
            input_boxes = box.int()
            masks = None
            if input_boxes.size(0) > 0:
                masks, _, _ = self.sam2_predictor.predict(
                    point_coords=None,
                    point_labels=None,
                    box=input_boxes,
                    multimask_output=False,
                )
            if (masks is not None) and len(masks.shape) == 3:
                masks = np.expand_dims(masks, axis=0)
            img_overlay = np.array(cv_img)

            # Draw bounding boxes and publish results
            for i in range(len(box)):
                label = f"{self.model.names[int(cat[i])]} {conf[i] * 100:.1f}%"
                color = [random.randint(0, 255) for _ in range(3)]
                c1, c2 = (int(box[i][0]), int(box[i][1])), (int(box[i][2]), int(box[i][3]))
                cv2.rectangle(img_overlay, c1, c2, color, 3, lineType=cv2.LINE_AA)
                cv2.putText(img_overlay, label, (c1[0], c1[1] - 2), 0, 1, (225, 255, 255), 1, lineType=cv2.LINE_AA)
                img_overlay = self.show_mask(masks[i], img_overlay, plt.gca()) if masks is not None else None

            self.publish_bbox(box, cat, conf, timestamp, masks)
            self.yolo_vis_pub.publish(self.bridge.cv2_to_imgmsg(img_overlay, encoding="bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in inference: {e}")
        finally:
            self.processing = False  # Mark as done processing
        
    def publish_bbox(self, box, cat, conf, timestamp, masks=None):

        bbox_msg = Detection2DArray()
        bbox_msg.detections = []

        for i in range(len(box)):
            bbox = Detection2D()
            bbox_msg.header.stamp = timestamp
            bbox.bbox.center.x = (int(box[i][0]) + int(box[i][2])) / 2
            bbox.bbox.center.y = (int(box[i][1]) + int(box[i][3])) / 2
            bbox.bbox.size_x = abs(int(box[i][2]) - int(box[i][0]))
            bbox.bbox.size_y = abs(int(box[i][3]) - int(box[i][1]))

            # Create object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(cat[i])
            hypothesis.score = conf[i]

            bbox.results.append(hypothesis)

            # Convert and attach mask if available
            assert(len(masks) == len(box))
            if masks is not None and len(masks) > i:
                mask = masks[i]
                mask = mask.squeeze(0) if mask.shape[0] == 1 else mask
                mask = PIL.Image.fromarray((mask * 255).astype(np.uint8))
                mask = self.bridge.cv2_to_imgmsg(np.array(mask), encoding="mono8")

                # mask_mono8 = (masks[i] * 255).astype(np.uint8)
                # mask_msg = self.bridge.cv2_to_imgmsg(mask_mono8, encoding="mono8")
                bbox.source_img = mask # Ensure Detection2D supports this field

            bbox_msg.detections.append(bbox)

        bbox_msg.header.stamp = timestamp
        bbox_msg.header.frame_id = 'map'
        self.yolo_detect_pub.publish(bbox_msg)

    def show_mask(self, mask, img, ax=None, random_color=True, borders=True):
        if mask is None or img is None:
            print("Error: mask or img is None!")
            return

        # Ensure mask is 8-bit binary
        mask = mask.astype(np.uint8) * 255  
        mask_image = np.zeros((*mask.shape, 3), dtype=np.uint8)  # RGB blank image
        
        # Generate color
        color = np.random.rand(3) if random_color else np.array([1, 0, 0])  # Default red
        mask_image[mask > 0] = (color * 255).astype(np.uint8)  # Apply color

        # Resize mask_image if dimensions don't match
        mask_image = mask_image[0]
        if mask_image.shape[:2] != img.shape[:2]:
            mask_image = cv2.resize(mask_image, (img.shape[1], img.shape[0]))


        # Ensure img is 3-channel
        if len(img.shape) == 2 or img.shape[2] == 1:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Ensure mask_image is 3-channel
        if mask_image.shape[2] == 1:
            mask_image = cv2.cvtColor(mask_image, cv2.COLOR_GRAY2BGR)
        # Blend mask with image
        alpha = 0.6
        overlay = cv2.addWeighted(img, 1, mask_image, alpha, 0)

        return overlay

if __name__ == '__main__':
    rospy.init_node('sam2', anonymous=True)
    sam2 = SAM2()
    rospy.spin()