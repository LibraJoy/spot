import spot_spot as spot
import spot_webrtc
import spot_move
import trimesh
import os
import cv2
import numpy as np
import time

from w_detect import search_rect2
from trial import improve_corners
from trial import trace_corners

ang = 15.0
dist = 1.0

# circular movement
def action():
  mdl = trimesh.load("../cv/cad/sliding_m.obj")
  spot.stand()
  spot.set_screen('mech_full')
  spot.ptzSet(144.0, 0.0, 1.0)
  time.sleep(1.0)

  spot.move(1,1)
  time.sleep(1.0)
  for c in range(2):
    if not spot.move(0, 1.0):
      break
    while True:
      time.sleep(1.0)
      spot.ptzSet(144.0, 12.0, 1.0)
      corners = search_rect2(img2, mdl, hfv=31.85/spot.zoom, pan=spot.pan, tilt=spot.tilt)
      if len(corners) >= 0:
        break
    improve_corners(corners, spot, [5.0, 10.0])
    if c == 0:
      trace_corners(corners, spot, zoom=18.0, saveImage=False)
    
    spot.ptzSet(144.0, 0.0, 1.0)
    time.sleep(1.0)
    
    if not spot.move(0, 1.0):
        break
    if not spot.rotate(90):
        break
    if not spot.move(0, 4):
        break
    if not spot.move(2, 0):
        break
    if not spot.rotate(-90):
        break
    if not spot.move(-4, 0):
        break
  spot.move(-1,-1)


      

if __name__ == '__main__':
  spot_move.moveSpot(movement=action)
  time.sleep(1.0)
  spot_move.endSpot()
