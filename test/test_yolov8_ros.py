
import PIL.Image
import cv2
import PIL
import torch 
import numpy as np

from ultralytics import YOLO
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
# import vision_msgs.msg
class yolo_seg:
    def __init__(self):
        self.w_scaled = 640
        self.h_scaled = 640
        self.model = YOLO("/home/xiaoyang/Downloads/yolov8m-seg.pt")
        self.img_resized = None
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber('spot_image', Image, self.image_callback)
        self.yolo_vis_pub = rospy.Publisher('yolo/visualization', Image, queue_size=10)
        self.yolo_detect_pub = rospy.Publisher('yolo/detection', Detection2DArray, queue_size=10)
        self.img_data = None

    def plot(self, results):
        for i, r in enumerate(results):
            # Plot results image
            im_bgr = r.plot()  # BGR-order numpy array
            im_rgb = PIL.Image.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

            # Show results to screen (in supported environments)
            r.show()

            # Save results to disk
            r.save(filename=f"results{i}.jpg")

    def publish_results(self, results):
        detection_array = Detection2DArray()
        for i, r in enumerate(results): # i always 0
            # publish visualizaiton image
            im_bgr = r.plot()  # BGR-order numpy array
            cv_img = cv2.cvtColor(np.array(im_bgr), cv2.COLOR_RGB2BGR)
            ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            self.yolo_vis_pub.publish(ros_img)

            # publish detection results
            detection = Detection2D()
            for j in range(r.boxes.cls.shape[0]): # number of bounding boxes in current frame
                detection.header.stamp = self.img_data.header.stamp
                detection.header.frame_id = "yolo_result"
                result = ObjectHypothesisWithPose()
                result.id = int(r.boxes.cls[j])
                result.score = int(r.boxes.conf[j])
                detection.results.append(result)
                detection.bbox.center.x = r.boxes.xywh[j][0]
                detection.bbox.center.y = r.boxes.xywh[j][1]
                detection.bbox.size_x = r.boxes.xywh[j][2]
                detection.bbox.size_y = r.boxes.xywh[j][3]
            detection_array.detections.append(detection)
            self.yolo_detect_pub.publish(detection_array)
        
 

    def image_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_data = data
        except CvBridgeError as e:
            print(e)

        # self.img_resized = cv2.resize(cv_img, (self.w_scaled, self.h_scaled))

        # # put model and image to gpu
        # self.model.to("cuda")

        # # Convert the OpenCV image to a PyTorch tensor
        # tensor_img = torch.from_numpy(self.img_resized).float().div(255).permute(2, 0, 1).unsqueeze(0)

        # # Move the tensor to the GPU
        # tensor_img = tensor_img.to("cuda")

        results = self.model.predict(source=cv_img, save=False, save_txt=False, stream=True, verbose=False)
        # self.plot(results)
        self.publish_results(results)

        # try:
        #     ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        #     self.yolo_img_pub.publish(ros_img)
        # except CvBridgeError as e:
        #     print(e)

    # make boundinb box and semantic mask msg from yolo results->Detection2DArray
    def make_msg(self, results):
        print("make_msg")
        detection_array = Detection2DArray()
        for i, r in enumerate(results):
            print("get r")
            detection = Detection2D()
            detection.header.stamp = self.img_data.header.stamp
            detection.header.frame_id = "yolo_result"
            for j in range(r.boxes.cls.shape[0]): # number of bounding boxes in current frame
                box = BoundingBox2D()
                box.results[0].id = r.boxes.cls[j]
                box.results[0].score = r.boxes.conf[j]
                box.center.x = r.boxes.xywh[j][0]
                box.center.y = r.boxes.xywh[j][1]
                box.size_x = r.boxes.xywh[j][2]
                box.size_y = r.boxes.xywh[j][3]
                detection.results.append(box)
            detection_array.detections.append(detection)
        return detection_array

if __name__ == '__main__':
    rospy.init_node('yolo_seg', anonymous=True)
    yolo_seg = yolo_seg()
    rospy.spin()