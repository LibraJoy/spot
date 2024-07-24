
import PIL.Image
import cv2
import PIL
import torch 
import numpy as np

from ultralytics import YOLO
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class yolo_seg:
    def __init__(self):
        self.w_scaled = 640
        self.h_scaled = 640
        self.model = YOLO("/home/xiaoyang/Downloads/yolov8m-seg.pt")
        self.img_resized = None
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber('spot_image', Image, self.image_callback)
        self.yolo_img_pub = rospy.Publisher('yolo_result_image', Image, queue_size=10)



    def predict(self, source):
        results = self.model.predict(source=source, save=False, save_txt=False, stream=True)
        return results

    def plot(self, results):
        for i, r in enumerate(results):
            # Plot results image
            im_bgr = r.plot()  # BGR-order numpy array
            im_rgb = PIL.Image.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

            # Show results to screen (in supported environments)
            r.show()

            # Save results to disk
            r.save(filename=f"results{i}.jpg")

    def publish_vis(self, results):
        for i, r in enumerate(results):
            im_bgr = r.plot()  # BGR-order numpy array
            cv_img = cv2.cvtColor(np.array(im_bgr), cv2.COLOR_RGB2BGR)
            try:
                ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                self.yolo_img_pub.publish(ros_img)
            except CvBridgeError as e:
                print(e)

    def image_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # self.img_resized = cv2.resize(cv_img, (self.w_scaled, self.h_scaled))

        # # put model and image to gpu
        # self.model.to("cuda")

        # # Convert the OpenCV image to a PyTorch tensor
        # tensor_img = torch.from_numpy(self.img_resized).float().div(255).permute(2, 0, 1).unsqueeze(0)

        # # Move the tensor to the GPU
        # tensor_img = tensor_img.to("cuda")

        results = self.predict(cv_img)
        # self.plot(results)
        self.publish_vis(results)

        # try:
        #     ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        #     self.yolo_img_pub.publish(ros_img)
        # except CvBridgeError as e:
        #     print(e)

if __name__ == '__main__':
    rospy.init_node('yolo_seg', anonymous=True)
    yolo_seg = yolo_seg()
    rospy.spin()