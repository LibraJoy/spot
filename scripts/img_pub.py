#!/usr/bin/env python3
import spot.spot_spot as spot
import spot.spot_webrtc as spot_webrtc
from spot.webrtc_client import WebRTCClient
from aiortc import RTCConfiguration
import asyncio
import cv2
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

import threading

p = None
webrtc_thread = None
shutdown_flag = None
bridge = CvBridge()

# tgtdir = 'images/' + time.strftime('%Y-%m-%d-%H-%M-%S')
# os.makedirs(tgtdir, exist_ok=True)

def startMonitor(hostname, robot, process=spot_webrtc.captureT):
  global webrtc_thread
  global shutdown_flag
  spot_webrtc.frameCount = 0
  spot_webrtc.frameR = None
  # spot.set_screen('mech_full')  # PTZ camera
  #spot.set_screen('digi_full')
  spot.set_screen('pano_full') # for searching window
  #spot.set_screen('c0')
#   spot.stand()
  # Suppress all exceptions and log them instead.
  #sys.stderr = InterceptStdErr()

  spot_webrtc.frameCount = 0
  spot_webrtc.frameR = None
  # set up webrtc thread (capture only)
  if webrtc_thread is None:
    shutdown_flag = threading.Event()
    webrtc_thread = threading.Thread(
      target=spot_webrtc.start_webrtc, args=[shutdown_flag, hostname, robot.user_token, process],
      daemon=True)

  # start webrtc thread
  webrtc_thread.start()

  # rate = rospy.Rate(20)
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    #print(spot_webrtc.frameCount)
    if not webrtc_thread.is_alive():
      break
    elif spot_webrtc.frameCount == 0:
      time.sleep(0.1)
      tm1 = time.time()
      print("-------------------------- frame count = 0 ----------------------")
      if spot_webrtc.frameR is None:
        print("-------------------- NO FRAME RECEIVED FROM QUEUE --------------------")
    else:
      img = spot_webrtc.cvImage.copy()
      # import pdb;pdb.set_trace()
      # for i in range(img.shape[1]):
      #   if i>1200:
      #     img[:,i,:] = 0
  
      # print(rospy.Time.now())
      # cv2.imshow("Image", img)

      publish_image_to_ros(img)

      # for i in range(6):
      #   time.sleep(0.1)
      #   cv2.imwrite(tgtdir + '/{:02d}.jpg'.format(i), img)
      #   print('image saved')
      # end of else
    rate.sleep()

    c = cv2.waitKey(1)
    if c == 27:
      break

def publish_image_to_ros(cv_img):
  global bridge
  try:
    ros_img = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
    ros_img.header.stamp = rospy.Time.now()
    img_pub.publish(ros_img)
  except CvBridgeError as e:
    print(e)

def endSpot():
  global p
  global webrtc_thread
  if webrtc_thread is not None:
    # stop webrtc capture thread        
    shutdown_flag.set()
    try:
      webrtc_thread.join()
      #print('Successfully saved webrtc images to local directory.')
    except KeyboardInterrupt:
      shutdown_flag.set()
      webrtc_thread.join(timeout=3.0)

  time.sleep(1.0)      
#   spot.sit()

  cv2.destroyAllWindows()
  # close webRTC process
  if p is not None:
    p.terminate()
    p.join()

# main loop    
def publish():
  global p
#   spot.stand()
  spot.set_screen('pano_full')
  # spot.set_screen('mech_full')

  startMonitor(spot.hostname, spot.robot)


if __name__ == '__main__':

  while True:
    if spot.connect():
        print("spot connected")
        break
    else:
      print("connnection failed")

  rospy.init_node('img_publisher', anonymous=True)
  img_pub = rospy.Publisher('spot_image', Image, queue_size=10)

  try:
    publish()
    time.sleep(1.0)
  except KeyboardInterrupt:
    endSpot()
    rospy.signal_shutdown('Keyboard interrupt')