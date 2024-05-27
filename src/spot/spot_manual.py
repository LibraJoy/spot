import numpy as np
import cv2
import time

import spot_spot as spot


# start SPOT
spot.connect()

img = spot.ptzImage()
cv2.imshow('camera', img)

spot.rotate(45.0)
spot.rotate(-90.0)
spot.rotate(45.0)

spot.move(-0.5, 0.5)
spot.move(1.0, 0.0)
spot.move(-0.5, -0.5)

spot.ptzTilt(spot.tilt+30)
spot.ptzPan(spot.pan+50)
time.sleep(1)
img = spot.ptzImage()
cv2.imshow('camera #2', img)

print('lighting set(0.1,0.2,0.3,0.4)-2sec->restore original brightness')
org = spot.getLight()
spot.setLight([0.1, 0.2, 0.3, 0.4])
spot.getLight()
time.sleep(2.0)
spot.setLight(org)
spot.getLight()

print('ptz camera move')

spot.ptzTilt(spot.tilt-15)
spot.ptzPan(spot.pan+100)
#spot.ptzZoom(spot.zoom+5.0)
spot.ptzZoom(30)
time.sleep(5)
img = spot.ptzImage()
cv2.imshow('camera #3', img)

cv2.waitKey(0)
cv2.destroyAllWindows()

# end SPOT
spot.disconnect()   
