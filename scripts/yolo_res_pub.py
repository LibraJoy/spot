#!/usr/bin/env python3
import spot.spot_spot as spot
import spot.spot_webrtc as spot_webrtc
import spot.spot_move as spot_move
import cv2
import asyncio
import multiprocessing as mp
import sys
import random
import time
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
# model_path = os.path.join(pkg_path, 'src/spot', 'yolov7.pt')
model_path = os.path.join(pkg_path, 'src/spot', 'yolov7-tiny.pt')

bridge = CvBridge()

def action():
  # tgtdir = 'images/' + time.strftime('%Y-%m-%d-%H-%M-%S')
  # os.makedirs(tgtdir, exist_ok=True)

  spot.stand()
  spot.set_screen('pano_full')

  time.sleep(1)
  
  device = 'cpu'
  if torch.cuda.is_available():
    device = 'cuda:0'
  torch.device(device)

  # model = yolov7.load('yolov7.pt') # ... COCO
  model = yolov7.load(model_path)
  
  # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
  # out = cv2.VideoWriter('outputs.mp4', fourcc, 5, (1280,720), True)

  # cv2.namedWindow('yolo_results')

  while True:
  # while not rospy.is_shutdown():
    rgb = spot_webrtc.rgbImage.copy()
    img = spot_webrtc.cvImage.copy()

    results = model(rgb)
    p = results.pred[0]
    box = p[:,:4] # bbox start and end points
    conf = p[:,4] # condidence
    cat = p[:,5] # category id
    tl = 3
    for i in range(len(box)):
      label = '{} {:.1f}%'.format(model.names[int(cat[i])], conf[i]*100.0)
      # print(label)
      color = [random.randint(0, 255) for _ in range(3)]
      c1, c2 = (int(box[i][0]), int(box[i][1])), (int(box[i][2]), int(box[i][3]))
      cv2.rectangle(img, c1, c2, color, tl, lineType=cv2.LINE_AA) # object bounding box
      tf = max(tl -1, 1)
      t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
      c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
      cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled box for labels
      cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

    # for i in range(6):
    #   time.sleep(1.0)
    #   cv2.imwrite(tgtdir + '/{:02d}.jpg'.format(i), img)
    #   print('image saved')

    # cv2.imshow('yolo_results', img)

    raw_img = spot_webrtc.cvImage.copy()

    publish_image_to_ros(raw_img)
    publish_yolo_img_to_ros(img)
    publish_bbox(box, cat, conf)
    publish_sem_label(model, cat)

    # out.write(img)
    # c = cv2.waitKey(1)
    # if c == 27:
    #   break

  # out.release()

def publish_image_to_ros(cv_img):
  global bridge
  try:
    ros_img = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
    img_pub.publish(ros_img)
  except CvBridgeError as e:
    print(e)

def publish_yolo_img_to_ros(cv_img):
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


if __name__ == '__main__':
  img_pub = rospy.Publisher('spot_image', Image, queue_size=10)
  yolo_img_pub = rospy.Publisher('yolo_image', Image, queue_size=10)
  bbox_pub = rospy.Publisher('yolo_bbox', Detection2DArray, queue_size=10)
  label_pub = rospy.Publisher('yolo_label', SemanticLabel, queue_size=10)
  rospy.init_node('yolo_publisher', anonymous=True)

  if len(sys.argv) > 1:
    #print(sys.argv[1])
    spot.set_hostname(sys.argv[1])

  while True:
    if spot.connect():
      break

  try:
    spot.stand()
    spot.set_screen('pano_full')

    spot_move.startMonitor(spot.hostname, spot.robot, movement=action)

  except KeyboardInterrupt:
      spot.endSpot()
      rospy.signal_shutdown('Keyboard interrupt')
      print("Program terminated.")
