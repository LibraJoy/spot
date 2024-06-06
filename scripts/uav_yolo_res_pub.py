#!/usr/bin/env python3
import cv2
import random
import torch
import yolov7
import os
import rospkg

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
from spot.msg import SemanticLabel

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('spot')
model_path = os.path.join(pkg_path, 'src/spot', 'yolov7.pt')

bridge = CvBridge()
model = None

def load_yolo_model(model_path):
  global model
  device = 'cpu'
  if torch.cuda.is_available():
    device = 'cuda:0'
  torch.device(device)
  model = yolov7.load(model_path)
  return model

def publish_yolo_res_img_to_ros(cv_img):
  global bridge
  try:
    ros_img = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
    yolo_img_pub.publish(ros_img)
  except CvBridgeError as e:
    print(e)

def publish_bbox(box, cat, conf):
  bbox_msg = Detection2DArray()
  bbox_msg.detections = []

  for i in range(len(box)):
    bbox = Detection2D()

    bbox.bbox.center.x = (int(box[i][0]) + int(box[i][2])) / 2
    bbox.bbox.center.y = (int(box[i][1]) + int(box[i][3])) / 2
    bbox.bbox.size_x = abs(int(box[i][2]) - int(box[i][0]))
    bbox.bbox.size_y = abs(int(box[i][3]) - int(box[i][1]))

    hypothesis = ObjectHypothesisWithPose()
    hypothesis.id = int(cat[i])
    hypothesis.score = conf[i]
    bbox.results.append(hypothesis)
    
    bbox_msg.detections.append(bbox)
    bbox_msg.header.stamp = rospy.Time.now()
    bbox_msg.header.frame_id = 'yolo_bbox'

  bbox_pub.publish(bbox_msg)

def publish_sem_label(model, cat):
  labels_msg = SemanticLabel()
  labels_msg.ids = []
  labels_msg.labels = []

  for i in range(len(cat)):
    cat_id = int(cat[i])
    label = model.names[int(cat[i])]

    labels_msg.ids.append(cat_id)
    labels_msg.labels.append(label)

  label_pub.publish(labels_msg)

def img_cb(msg):
  if msg is None:
    rospy.loginfo("No image received, waiting ...")

  ros_img = msg

  cv_img = bridge.imgmsg_to_cv2(ros_img, desired_encoding='passthrough').copy()
  rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

  results = model(rgb_img)
  p = results.pred[0]
  box = p[:,:4] # bbox start and end points
  conf = p[:,4] # condidence
  cat = p[:,5] # category id
  tl = 3
  for i in range(len(box)):
    label = '{} {:.1f}%'.format(model.names[int(cat[i])], conf[i]*100.0)
    color = [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(box[i][0]), int(box[i][1])), (int(box[i][2]), int(box[i][3]))
    cv2.rectangle(cv_img, c1, c2, color, tl, lineType=cv2.LINE_AA) # object bounding box
    tf = max(tl -1, 1)
    t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
    c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
    cv2.rectangle(cv_img, c1, c2, color, -1, cv2.LINE_AA)  # filled box for labels
    cv2.putText(cv_img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
  
  publish_bbox(box, cat, conf)
  publish_yolo_res_img_to_ros(cv_img)
  publish_sem_label(model, cat)


if __name__ == '__main__':
  try:
    model = load_yolo_model(model_path)
    rospy.init_node('yolo_publisher', anonymous=True)
    img_sub = rospy.Subscriber('camera/color/image_raw', Image, img_cb)
    yolo_img_pub = rospy.Publisher('uav/yolo_image', Image, queue_size=10)
    bbox_pub = rospy.Publisher('uav/yolo_bbox', Detection2DArray, queue_size=10)
    label_pub = rospy.Publisher('uav/yolo_label', SemanticLabel, queue_size=10)
    rospy.spin()
  except rospy.ROSInternalException:
    pass
