

import spot_spot as spot
import cv2

spot.connect()
# camera 'frontleft', 'frontright', 'left', 'right', 'back'
img, dpth = spot.depthData('right')

dp2 = cv2.normalize(dpth, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX)
cv2.imshow('img', img)
cv2.imshow('depth', dp2)
cv2.waitKey(0)
