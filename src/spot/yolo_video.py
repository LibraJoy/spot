import spot_spot as spot
import spot_webrtc
import spot_move
import cv2
import asyncio
import multiprocessing as mp
import sys
import random
import time
import torch
import yolov7
import os

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def action():
  tgtdir = 'images/' + time.strftime('%Y-%m-%d-%H-%M-%S')
  os.makedirs(tgtdir, exist_ok=True)

  spot.stand()
  spot.set_screen('pano_full')

  time.sleep(1)
  
  device = 'cpu'
  if torch.cuda.is_available():
    device = 'cuda:0'
  torch.device(device)

  model = yolov7.load('yolov7.pt') # ... COCO
  
  # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
  # out = cv2.VideoWriter('outputs.mp4', fourcc, 5, (1280,720), True)

  while True:
    rgb = spot_webrtc.rgbImage.copy()
    img = spot_webrtc.cvImage.copy()
    if img is None or img.size == 0:
      print("----------------- NO IMG RECEIVED FROM WEBRTC -----------------")
      continue
      
    results = model(rgb)
    p = results.pred[0]
    box = p[:,:4]
    conf = p[:,4]
    cat = p[:,5]
    tl = 3
    for i in range(len(box)):
      label = '{} {:.1f}%'.format(model.names[int(cat[i])], conf[i]*100.0)
      print(label)
      color = [random.randint(0, 255) for _ in range(3)]
      c1, c2 = (int(box[i][0]), int(box[i][1])), (int(box[i][2]), int(box[i][3]))
      cv2.rectangle(img, c1, c2, color, tl, lineType=cv2.LINE_AA)
      tf = max(tl -1, 1)
      t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
      c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
      cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
      cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

    for i in range(6):
      time.sleep(1.0)
      cv2.imwrite(tgtdir + '/{:02d}.jpg'.format(i), img)
      print('saved image')

    cv2.imshow('image', img)

    # out.write(img)
    c = cv2.waitKey(1)
    if c == 27:
      break
  # out.release()

if __name__ == '__main__':
  if len(sys.argv) > 1:
    #print(sys.argv[1])
    spot.set_hostname(sys.argv[1])
  while True:
    if spot.connect():
      break
  spot.stand()
  spot.set_screen('pano_full')

  spot_move.startMonitor(spot.hostname, spot.robot, movement=action)

  spot_move.endSpot()
