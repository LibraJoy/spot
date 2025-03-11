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


from sensor_msgs.msg import Image
import rospy


class SAM2:
    def __init__(self):
        self.w_org = 1280
        self.h_org = 720
        # self.model_path = "/home/user/spot_ws/src/spot/models/yolov7.pt"
        self.model_path = "/home/user/spot_ws/src/spot/models/best.7.10.pt"
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
        
        self.img_resized = None
        self.bridge = CvBridge()

        if device.type == "cuda":
            # use bfloat16 for the entire notebook
            torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
            # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
            if torch.cuda.get_device_properties(0).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True
        elif device.type == "mps":
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
        sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=device)
        self.sam2_predictor = SAM2ImagePredictor(sam2_model)
        self.id=0
        # print("SAM2 model loaded successfully.")
        self.bag="new"
        if self.bag=="new":
            self.img_sub = rospy.Subscriber('/spot_image', Image, self.image_callback)
        else:
            self.img_sub = rospy.Subscriber('/yolov7/yolov7/visualization', Image, self.image_callback)
        # self.img_sub = rospy.Subscriber('/spot_image', Image, self.image_callback)
        self.yolo_vis_pub = rospy.Publisher('sam/visualization', Image, queue_size=10)
        self.yolo_detect_pub = rospy.Publisher('sam/detection', Detection2DArray, queue_size=10)
        self.yolo_mask_pub = rospy.Publisher('sam/mask', Image, queue_size=10)
        self.img_data = None
        





    def publish_results(self, results, names, timestamp):

        def show_mask(mask, img, ax=None, random_color=True, borders=True):
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

            # Save overlayed image
            # cv2.imwrite('output_image1.png', overlay)
            # print("Overlay saved as output_image1.png")
            return overlay



        start_time = time.time()

        tl=1
        self.id+=1
        p = results.pred[0]
        box, conf, cat = p[:, :4], p[:, 4], p[:, 5]  # Extract all at once
        input_boxes = box.int() 

        img_overlay = np.array(self.img_data, copy=False)




        # output_filename = f"image{self.id}.png"
        # output_path = os.path.join(os.path.dirname(__file__), self.bag+'input', output_filename)
        # os.makedirs(os.path.dirname(output_path), exist_ok=True)
        # cv2.imwrite(output_path, img_overlay)


        image = cv2.cvtColor(img_overlay, cv2.COLOR_BGR2RGB)
        self.sam2_predictor.set_image(image)
        masks=None
        mask_base = np.zeros(img_overlay.shape[:2], dtype=np.uint8)

        if input_boxes.size(0) > 0:
            masks, scores, _ = self.sam2_predictor.predict(
                point_coords=None,
                point_labels=None,
                box=input_boxes,
                multimask_output=False,
            )
            
            end_time = time.time()
            execution_time = end_time - start_time
            print(f"SAM execution time: {execution_time:.6f} seconds")

            for mask in masks:
                mask = mask.squeeze(0) if mask.shape[0] == 1 else mask
                mask_base = np.where(mask_base + mask > 0, 1, 0)
                img_overlay = show_mask(mask, img_overlay, plt.gca())
        # end_time = time.time()
        # execution_time = end_time - start_time
        # print(f"SAM execution time: {execution_time:.6f} seconds")

        for (x1, y1, x2, y2), conf_score, category in zip(input_boxes, conf, cat):
            label = f"{names[int(category)]} {conf_score * 100:.1f}%"
            color = (0, 0, 255)  # Red
            c1, c2 = (x1, y1), (x2, y2)
            c1 = tuple(map(int, c1))  # Convert c1 to tuple of integers
            c2 = tuple(map(int, c2))  # Convert c2 to tuple of integers
            cv2.rectangle(img_overlay, c1, c2, color, tl, lineType=cv2.LINE_AA)

            tf = max(tl - 1, 1)
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = (c1[0] + t_size[0], c1[1] - t_size[1] - 3)
            cv2.rectangle(img_overlay, c1, c2, color, -1, cv2.LINE_AA)
            cv2.putText(img_overlay, label, (c1[0], c1[1] - 2), 0, tl / 3, (225, 255, 255), thickness=tf, lineType=cv2.LINE_AA)
        
        
        
        # output_filename = f"image{self.id}.png"
        # output_path = os.path.join(os.path.dirname(__file__), self.bag+'output', output_filename)
        # os.makedirs(os.path.dirname(output_path), exist_ok=True)
        # cv2.imwrite(output_path, img_overlay)




        ros_img = self.bridge.cv2_to_imgmsg(img_overlay, encoding="bgr8")

        mask_base = (mask_base * 255).astype(np.uint8)
        mask_base = PIL.Image.fromarray(mask_base)
        mask_base = self.bridge.cv2_to_imgmsg(np.array(mask_base), encoding="mono8")
        self.yolo_mask_pub.publish(mask_base)


        self.yolo_vis_pub.publish(ros_img)

        self.publish_bbox(timestamp, box, cat, conf, names, masks)


    def publish_bbox(self, timestamp, box, cat, conf,names,masks=None):

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
        bbox_msg.header.frame_id = 'sams2yolo_bbox'
        self.yolo_detect_pub.publish(bbox_msg)

    def image_callback(self, data):
        try:
            # Convert ROS image to OpenCV format
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_data = cv_img  # Store the image data
        except CvBridgeError as e:
            print(e)

        # get timestamp from data
        timestamp = data.header.stamp

        # Perform detection using YOLOv7 model
        yolo_start_time = time.time()
        results = self.model(cv_img)  # Get predictions from YOLOv7
        names = self.model.names
        yolo_end_time = time.time()
        execution_time = yolo_end_time - yolo_start_time
        print(f"YOLO execution time: {execution_time:.6f} seconds")

        # print("Detection results:", results)
        self.publish_results(results,names, timestamp)  # Publish the results (boxes, labels, etc.)

if __name__ == '__main__':
    rospy.init_node('sam2', anonymous=True)
    sam2 = SAM2()
    rospy.spin()
