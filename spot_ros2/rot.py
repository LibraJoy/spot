import spot_spot as spot
import spot_move
import spot_webrtc
import cv2
import time

# spot_webrtc.cvImage ... PTZ camera image (RTC protocol)
# stand -> rotate -> sit
def action():
  spot.stand()
  time.sleep(1.0)
  img = spot_webrtc.cvImage.copy()
  cv2.imshow('image', img)
  cv2.waitKey(1)
  spot.rotate(90)
  time.sleep(1.0)
  img = spot_webrtc.cvImage.copy()
  cv2.imshow('image', img)
  cv2.waitKey(1)
  spot.rotate(-90)
  cv2.waitKey(1)
  img = spot_webrtc.cvImage.copy()
  cv2.imshow('image', img)
  spot.sit()
  time.sleep(1.0)
  img = spot_webrtc.cvImage.copy()
  cv2.imshow('image', img)
  
if __name__ == '__main__':
  spot_move.moveSpot(movement=action)
  spot_move.endSpot()
