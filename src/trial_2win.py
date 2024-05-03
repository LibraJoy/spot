import spot_spot as spot
import spot_webrtc
import trial
from trial import search_rect,improve_corners, trace_corners

import asyncio
import cv2
import numpy as np
import threading
import time
import multiprocessing as mp
import math
import json
from pylsd import lsd
import argparse
import os

def monitor(hostname, robot, process=spot_webrtc.captureT, saveImage=False, maxCount=10):
  spot_webrtc.frameCount = 0
  spot.set_screen('mech_full')  # PTZ camera
  #spot.set_screen('digi_full')
  #spot.set_screen('pano_full') # for searching window
  #spot.set_screen('c0')
  spot.stand()
  spot.ptzSet(144.0, 9.0, 1.0)
  # Suppress all exceptions and log them instead.
  #sys.stderr = InterceptStdErr()

  spot_webrtc.frameCount = 0
  # set up webrtc thread (capture only)
  shutdown_flag = threading.Event()
  webrtc_thread = threading.Thread(
    target=spot_webrtc.start_webrtc, args=[shutdown_flag, hostname, robot.user_token, process],
    daemon=True)

  fc = 0
  maxCount *= 2
  
  #lt = 30
  #st1 = 10
  #st2 = 2
  #hfv = 31.85
  tanThr = math.tan(85.0/180.0 * math.pi)

  corners = []
  # start webrtc thread
  webrtc_thread.start()

  while True:
    #print(spot_webrtc.frameCount)
    if not webrtc_thread.is_alive():
      break
    elif spot_webrtc.frameCount == 0:
      time.sleep(0.1)
      tm1 = time.time()
      i = 0
    else:
      spot.ptzSet(144.0, 9.0, 1.0)
      time.sleep(1.0)
      img = spot_webrtc.cvImage.copy();
      mode = spot.get_screen()
      #cv2.imwrite(mode + '_' + str(int(spot.pan)) + '_' + str(int(spot.tilt)) + '_' + str(int(spot.zoom)) + '.jpg', img)
      #cv2.imshow('image', img)
            
      # image pyramid (only 1 reduction)
      h,w = img.shape[:2]
      img2 = cv2.resize(img,(w//2, h//2),cv2.INTER_AREA)
      h,w = img2.shape[:2]
      img4 = cv2.resize(img2,(w//2, h//2),cv2.INTER_AREA)
      h,w = img4.shape[:2]
      corners = search_rect(img4, spot)
      # end of else
      c = cv2.waitKey(1)
      if c == 27:
        break
      if len(corners) >= 1:
        time.sleep(1.0)
        # fine tune of PAN, TILT value
        improve_corners(corners, spot, [5.0, 10.0])
        #
        trace_corners(corners, spot, zoom = 18.0, saveImage=saveImage)
        tm2 = time.time()
        if tm2-tm1 > 5.0:
          # initial camera
          spot.ptzSet(144.0, 9.0, 1.0)
          if not (fc % 2):
            spot.rotate(-30)
          else:
            spot.rotate(30)
          fc += 1
          tm1 = tm2
          if fc >= maxCount:
            break
      
  # stop webrtc capture thread        
  shutdown_flag.set()
  try:
    webrtc_thread.join()
    cv2.destroyAllWindows()
    #print('Successfully saved webrtc images to local directory.')
  except KeyboardInterrupt:
    shutdown_flag.set()
    webrtc_thread.join(timeout=3.0)

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Lucid camera livestream and capture')
  parser.add_argument('-s', '--save', action='store_true')
  parser.add_argument('-ns', '--no-save', dest = 'save', action = 'store_false')
  parser.set_defaults(save=False)
  args = parser.parse_args()

  if args.save:
    trial.saveImage = True
    os.makedirs("images", exist_ok=True)
  while True:
    if spot.connect():
      spot.set_screen('mech_full')
      #spot.stand()
      # start webrtc monitoring process (just monitoring)
      p = mp.Process(target = spot_webrtc.monitor, args=(spot.hostname, spot.robot))
      p.start()
      monitor(spot.hostname, spot.robot)
      spot.ptzSet(144, 0, 1.0)
      spot.sit()
      spot.disconnect()
        
      # stop webrtc monitoring
      p.terminate()
      p.join()
      break
