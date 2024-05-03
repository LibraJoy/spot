import spot_spot as spot
import spot_webrtc

import threading
import time
import multiprocessing as mp
import asyncio
import cv2
import numpy as np
import math
import json
from pylsd import lsd

#print(sorted(glob.glob(args.input + '/*.jpg')))

saveImage = False

def line(p1, p2):
  # line passing 2 points
  # return ax + by + c = 0
  # (y2-y1)(x-x1)-(x2-x1)(y-y1)=0
  a = p2[1]-p1[1]             # y2-y1
  b = p1[0]-p2[0]             # x1-x2
  c = - a * p1[0] - b * p1[1]
  return a,b,c

def line_cross(l1, l2):
  # cross point of 2 lines
  # a1x + b1y + c1 = 0 ... l1[0,1,2]
  # a2x + b2y + c2 = 0 ... l2[0,1,2]
  # x = (b1c2 - b2c1) / (a1b2 - a2b1)
  # y = (a2c1 - a1c2) / (a1b2 - a2b1)
  d = l1[0]*l2[1] - l2[0]*l1[1]
  if abs(d) < 1.0e-6:
    return False, 0, 0
  x = (l1[1]*l2[2] - l2[1]*l1[2]) / d
  y = (l2[0]*l1[2] - l1[0]*l2[2]) / d
  return True, x, y

def chkCntSize(cnt, wt, ht):
  x,y,w,h = cv2.boundingRect(cnt)
  if w <= wt or h <= ht:
    return False
  else:
    return True
  
def chkCntSize(cnt, wt, ht):
  x,y,w,h = cv2.boundingRect(cnt)
  if w <= wt or h <= ht:
    return False
  else:
    return True
  
def chkCntSize2(cnt, wt, ht):
  x,y,w,h = cv2.boundingRect(cnt)
  if w >= wt or h >= ht:
    return False
  else:
    return True

def chkCntLocation(cnt, h1, h2):
  x,y,w,h = cv2.boundingRect(cnt)
  if y <= h1 or y+h >= h2:
    return False
  else:
    return True
  
def find_color(frame, p1, p2, lt, wt1, ht1, wt2, ht2, h1, h2):
  mask = cv2.inRange(frame, p1, p2)
  #cv2.imshow('mask', mask)
  cnts, hir = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  cnts = tuple(x for x in cnts if cv2.arcLength(x, True) > lt)
  # small
  cnts = tuple(x for x in cnts if chkCntSize(x, wt1, ht1))
  # large
  cnts = tuple(x for x in cnts if chkCntSize2(x, wt2, ht2))
  # location
  cnts = tuple(x for x in cnts if chkCntLocation(x, h1, h2))
  return cnts

def remove_included(apps):
  apps2 = []
  for a in apps:
    ad = {}
    ad['app'] = a
    x,y,w,h = cv2.boundingRect(a)
    ad['bb'] = [x,y, x+w, y+h]
    ad['included'] = False
    apps2.append(ad)
    
  for i in range(len(apps2)):
    a = apps2[i]
    for j in range(len(apps2)):
      if i == j:
        pass
      b = apps2[j]
      if (b['bb'][0] < a['bb'][0] and
          b['bb'][2] > a['bb'][2] and
          b['bb'][1] < a['bb'][1] and
          b['bb'][3] > a['bb'][3]):
        a['included'] = True
        break
  #print(apps)
  appsT = []
  for a in apps2:
    if not a['included']:
      appsT.append(a['app'])
  #print(appsT)
  return(appsT)

def search_rect(img, spot, st1=10, st2=4, lt=30, hfv=31.85):
  tanThr = math.tan(85.0/180.0 * math.pi)
  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
  apps = []
  corners = []
  h,w = img.shape[:2]
  #wt1 = w / st1
  #ht1 = h / st1
  wt1 = 10
  ht1 = 10
  wt2 = w / st2
  ht2 = h / st2
  h1 = h // 4
  h2 = (h * 3) // 4
  op = spot.pan
  ot = spot.tilt
  ll = (w // 2) / math.tan(hfv/180.0 * math.pi)

  #cv2.imshow('original', img)

  #
  # color quantization
  #

  # kmean clustering
  K = 16
  Z = img.reshape((-1,3))
  Z = np.float32(Z)
  #ret,label,center = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
  ret,label,center = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_PP_CENTERS)
  center = np.uint8(center)
  res = center[label.flatten()]
  img2 = res.reshape((img.shape))
  if saveImage:
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_kmean.jpg", img2)
  cv2.imshow('kmean', img2)

  # boundary of regions
  cnts = ()
  for i in range(K):
    p1 = center[i] - 1
    p2 = center[i] + 1
    cnts = cnts + find_color(img2, p1, p2, lt, wt1, ht1, wt2, ht2, h1, h2)
    
  imgg0 = np.zeros(shape=(h,w,1), dtype=np.uint8)  
  cv2.drawContours(imgg0, cnts, -1, (255), 2)
  if saveImage:
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_boundary.jpg", imgg0)
  cv2.imshow('boundary', imgg0)

  # merge boundary contours and remove long and short ones
  cnts, hir = cv2.findContours(imgg0, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  cnts = tuple(x for x in cnts if cv2.arcLength(x, True) > lt)
  # small
  cnts = tuple(x for x in cnts if chkCntSize(x, wt1, ht1))
  # large
  cnts = tuple(x for x in cnts if chkCntSize2(x, wt2, ht2))
  #
  cnts = tuple(x for x in cnts if chkCntLocation(x, h1, h2))
  
  imgg1 = np.zeros(shape=(h,w,1), dtype=np.uint8)  
  cv2.drawContours(imgg1, cnts, -1, (255), 1)
  if saveImage:
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_selected_contour.jpg", imgg1)
  cv2.imshow('3. Selected Contour', imgg1)
      
  #imgg2 = np.zeros(shape=(h,w,1), dtype=np.uint8)  
  imgg2 = img.copy()
  cnts, hirs = cv2.findContours(imgg1, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
  for ii in range(len(cnts)):
    cc = cnts[ii]
    hh = hirs[0][ii]
    if hh[3] != -1: # not outside of the region
      continue
    app = cv2.approxPolyDP(cc, 10, True)
    if len(app) == 4 and cv2.isContourConvex(app):
      x,y,wv,hv = cv2.boundingRect(app)
      xv = app[0][0][0]-(x+wv)
      yv = app[0][0][1]-y
      sum = xv*xv+yv*yv
      j = 0
      for i in range(1,4):
        xv = app[i][0][0]-(x+wv)
        yv = app[i][0][1]-y
        s = xv*xv+yv*yv
        if s < sum:
          sum = s
          j = i
      xv = app[(j+1)%4][0][0]-(x+wv)
      yv = app[(j+1)%4][0][1]-(y+hv)
      xv2 = app[(j+3)%4][0][0]-(x+wv)
      yv2 = app[(j+3)%4][0][1]-(y+hv)
      #print('1', xv*xv+yv*yv, '3',xv2*xv2+yv2*yv2)
      if xv2*xv2+yv2*yv2 > xv*xv+yv*yv:
        reverse = True
      else:
        reverse = False
      #print(x,y,wv,hv)
      #print(app, j, reverse)

      if j != 0:
        print('found contour not starting from top-right', j)
        #print(app.shape, app, 'shift: ', j)
        app = np.roll(app, -j, axis=0)
        #print('->app', app.shape, app)
      if reverse:
        app[[[1,3]]] = app[[[3,1]]]
      #print('->app', app.shape, app)
      # 06.28
      #print(app)
      yn = 0
      for i in range(1,4,2):
        #print(app[i][0],'-',app[(i+1)%4][0])
        xv = abs(app[(i+1)%4][0][0] - app[i][0][0])
        yv = abs(app[(i+1)%4][0][1] - app[i][0][1])
        #print(xv, yv, tanThr)
        if xv < 1:
          yn += 1
        else:
          r = float(yv) / float(xv)
          #print(r)
          if r >= tanThr:
            yn+= 1
      #print(yn)
      if yn == 2:
        apps.append(app)
  if len(apps) > 1:
    apps = remove_included(apps)
  #
  for a in apps:
    corner = []
    for c in range(4):
      x,y = a[c][0][:2]
      xx = x - w // 2
      yy = y - h // 2
      ax = math.atan(xx/ll) * 180.0 / math.pi
      ay = math.atan(yy/ll) * 180.0 / math.pi * -1
      #print(x,y, ax, ay)
      corner.append((ax+op, ay+ot))
      cv2.putText(imgg2, '({0:.1f}, {1:.1f})'.format(ax, ay),
                  (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1, cv2.LINE_AA)
    corners.append(corner)
  #print(corners)
  #print(apps)
  cv2.drawContours(imgg2, apps, -1, (0,255,0), 2)
  if saveImage:
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_Rectangle.jpg", imgg2)
  wn = '4. Rectangle'
  cv2.namedWindow(wn, cv2.WINDOW_NORMAL)
  cv2.imshow(wn, imgg2)
  return corners

def search_window(webrtc_thread):
  fc = 0

  #lt = 30
  #st1 = 10
  #st2 = 2
  #hfv = 31.85

  corners = []
  # start webrtc thread
  while True:
    #print(spot_webrtc.frameCount)
    if not webrtc_thread.is_alive():
      break
    elif spot_webrtc.frameCount == 0:
      time.sleep(0.1)
      tm1 = time.time()
      i = 0
    else:
      img = spot_webrtc.cvImage.copy();
      cv2.namedWindow('WebRTC', cv2.WINDOW_NORMAL)
      cv2.imshow('WebRTC', img)
      # image pyramid (only 1 reduction)
      h,w = img.shape[:2]
      img = cv2.resize(img,(w//2, h//2),cv2.INTER_AREA)
      h,w = img.shape[:2]
      #cv2.imshow('image', img)
      corners = search_rect(img, spot)
      # end of else
    c = cv2.waitKey(1)
    if c == 27:
      break
      
def monitor(hostname, robot, process=spot_webrtc.captureT):
  spot_webrtc.frameCount = 0
  #sdk = bosdyn.client.create_standard_sdk('Spot CAM Client')
  #spot_cam.register_all_service_clients(sdk)

  # Suppress all exceptions and log them instead.
  #sys.stderr = InterceptStdErr()

  shutdown_flag = threading.Event()
  webrtc_thread = threading.Thread(
    target=spot_webrtc.start_webrtc, args=[shutdown_flag, hostname, robot.user_token, process],
    daemon=True)
  webrtc_thread.start()

  search_window(webrtc_thread)
  '''
  while True:
    #print(spot_webrtc.frameCount)
    if not webrtc_thread.is_alive():
      break
    elif spot_webrtc.frameCount == 0:
      time.sleep(0.1)
      tm1 = time.time()
      i = 0
    else:
      img = spot_webrtc.cvImage.copy();
      h,w = img.shape[:2]
      img = cv2.resize(img,(w//2, h//2),cv2.INTER_AREA)
      h,w = img.shape[:2]
      cv2.imshow('image', img)
    c = cv2.waitKey(1)
    if c == 27:
      break
  '''
  try:
    webrtc_thread.join()
    #print('Successfully saved webrtc images to local directory.')
  except KeyboardInterrupt:
    shutdown_flag.set()
    webrtc_thread.join(timeout=3.0)

if __name__ == '__main__':
  while True:
    if spot.connect():
      break

  spot.set_screen('pano_full')
  p = mp.Process(target = monitor, args=(spot.hostname, spot.robot))
  p.start()

  time.sleep(10)
  
  spot.stand()

  for c in range(2):
    if not spot.move(0, 2):
      break
    if not spot.rotate(90):
      break
    if not spot.move(0, 4):
      break
    if not spot.rotate(90):
      break
    if not spot.move(0, 2):
      break
    if not spot.rotate(90):
      break
    if not spot.move(0, 4):
      break
    if not spot.rotate(90):
      break
  
  spot.sit()
  spot.disconnect()
  p.terminate()
  p.join()
