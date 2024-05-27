import spot_spot as spot
import spot_webrtc
import spot_move
import os
import cv2
import numpy as np
import time

ang = 15.0
dist = 1.0

# circular movement
def action():
  tgtdir = 'images/' + time.strftime('%Y-%m-%d-%H-%M-%S')
  os.makedirs(tgtdir, exist_ok=True)
  spot.ptzSet(144.0, 12.0, 1.0)
  time.sleep(1.0)
      
  for i in range(3):
    #time.sleep(1.0)
    img = spot_webrtc.cvImage.copy();
    cv2.imshow('image', img)
    cv2.waitKey(1)
    spot.rotate(ang)
    spot.move(0, -dist)
          
  for i in range(6):
    time.sleep(1.0)
    img = spot_webrtc.cvImage.copy();
    cv2.imshow('image', img)
    cv2.imwrite(tgtdir + '/{:02d}.jpg'.format(i), img)
    cv2.waitKey(1)
    spot.move(0, dist)
    spot.rotate(-ang)
          
  time.sleep(1.0)
  img = spot_webrtc.cvImage.copy();
  cv2.imwrite(tgtdir + '/{:02d}.jpg'.format(6), img)

  for i in range(3):
    #time.sleep(1.0)
    img = spot_webrtc.cvImage.copy();
    cv2.imshow('image', img)
    cv2.waitKey(1)
    spot.rotate(ang)
    spot.move(0, -dist)
  img = spot_webrtc.cvImage.copy();
  cv2.imshow('image', img)
  cv2.waitKey(1)

if __name__ == '__main__':
  spot_move.moveSpot(movement=action)
  time.sleep(1.0)
  spot_move.endSpot()
