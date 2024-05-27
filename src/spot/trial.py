import spot_spot as spot
import spot_webrtc

import trimesh
import asyncio
import cv2
import numpy as np
import threading
import time
import multiprocessing as mp
import math
from pylsd import lsd
import time
import argparse
import os
import sys

from  w_detect import search_rect2

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

def chkCntLocation(cnt, xmin, xmax, ymin, ymax):
  x,y,w,h = cv2.boundingRect(cnt)
  if x < xmin or x+w > xmax or y < ymin or y+h > ymax:
    return False
  else:
    return True
  
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
  
def find_color(frame, p1, p2, lt, wt1, ht1, wt2, ht2):
  mask = cv2.inRange(frame, p1, p2)
  #cv2.imshow('mask', mask)
  cnts, hir = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  cnts = tuple(x for x in cnts if cv2.arcLength(x, True) > lt)
  cnts = tuple(x for x in cnts if chkCntSize(x, wt1, ht1))
  #cnts = tuple(x for x in cnts if chkCntSize2(x, wt2, ht2))
  return cnts

def outlier(data):
  x = []
  y = []
  res = []
  for v in data:
    x.append(v[0])
    y.append(v[1])
  x = np.array(x)
  y = np.array(y)
  s_x = np.std(x)
  m_x = np.mean(x)
  s_y = np.std(y)
  m_y = np.mean(y)
  # standard deviations
  # https://www.mathsisfun.com/data/standard-normal-distribution-table.html
  # 68% ... 1.0
  # 70% ... 1.15
  # 79% ... 1.25
  # 87% ... 1.5
  # 96% ... 2.0
  # 98% ... 2.5
  thr = 1.5
  for v in data:
    if np.abs(v[0]-m_x) < s_x * thr and np.abs(v[1]-m_y) < s_y * thr:
      res.append(v)
  #print(len(data), len(res))
  return np.array(res)
  
# fine tune of PAN, TILT value from zoomed image
def improve_corners(corners, spot, zoom=[5.0], hfv=31.85):
  tanThr = math.tan(60.0/180.0*math.pi)

  for corner in corners:
    for ii in range(len(corner)):
      for z in zoom:
        hfv2 = hfv / z
        conr = corner[ii]
        spot.ptzSet(conr[0], conr[1], z)
        time.sleep(2.0)

        img = spot_webrtc.cvImage.copy()
        if saveImage:
          cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_corner.jpg", img)
        #cv2.imshow('corner', img)
        k = cv2.waitKey(1)
        h,w = img.shape[:2]
        img2 = cv2.resize(img,(w//2, h//2),cv2.INTER_AREA)
        h,w = img2.shape[:2]
        gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        sgmnts = lsd(gray)
        lg1 = []
        lg2 = []
        for s in sgmnts:
          pt1 = (int(s[0]), int(s[1]))
          pt2 = (int(s[2]), int(s[3]))
          ln = (s[0]-s[2], s[1]-s[3])
          l = ln[0]*ln[0]+ln[1]*ln[1]
          #print(pt1, pt2, ln, l)
          if l > 600.:
            a, b, c = line(pt1, pt2)
            #cv2.line(imgS, pt1, pt2, (0,0,255),2)
            aln0 = abs(ln[0])
            aln1 = abs(ln[1])
            if aln0 > tanThr * aln1:
              if pt1[0] > pt2[0]:
                lg1.append([pt2, pt1, ln, (a,b,c)])
              else:
                lg1.append([pt1, pt2, ln, (a,b,c)])
            elif aln1 > tanThr * aln0:
              if pt1[1] > pt2[1]:
                lg2.append([pt2, pt1, ln, (a,b,c)])
              else:
                lg2.append([pt1, pt2, ln, (a,b,c)])
            else:
              pass
        cnts = []
        if len(lg1) > 3 and len(lg2) > 3:
          for ln1 in lg1:
            for ln2 in lg2:
              ret, x, y = line_cross(ln1[3], ln2[3])
              if ret:
                #print(x,y)
                # check within 100px range
                if (x < 0 or x > w or y < 0 or y > h or
                    x < ln1[0][0]-100 or x > ln1[1][0]+100 or
                    y < ln2[0][1]-100 or y > ln2[1][1]+100):
                  continue
                # 06.28
                cx = (ln1[0][0]+ln1[1][0])//2
                cy = (ln2[0][1]+ln2[1][1])//2
                # order of points
                # 1--0
                # |  |
                # 2--3
                if (ii % 3) == 0:
                  if x < cx:
                    continue
                else:
                  if x > cx:
                    continue
                if (ii // 2) == 0:
                  if y > cy:
                    continue
                else:
                  if y < cy:
                    continue
                # 06.28
                cv2.circle(img2,(int(x),int(y)),5,(0,255,0),2)
                cnts.append((int(x),int(y)))
        cnts = np.array(cnts)
        cnts = outlier(cnts)
        time.sleep(1.0)
        if len(cnts) <= 3:
          continue
        x,y, bw, bh = cv2.boundingRect(cnts)
        #print(x, y, bw, bh)
        cv2.rectangle(img2,(x,y),(x+bw,y+bh),(0,0,255),3)
        cv2.imshow('intersection', img2)
        cv2.waitKey(1)
        mod = False
        xx = 0
        yy = 0
        if bw >= 10 and bw <= 200:
          mod = True
          xx = x + bw // 2 - w // 2
        if bh >= 10 and bh <= 200:
          mod = True
          yy = y + bh // 2 - h // 2
        if mod:
          ll = (w // 2) / math.tan(hfv2/180.0 * math.pi)
          ax = math.atan(xx/ll) * 180.0 / math.pi
          ay = math.atan(yy/ll) * 180.0 / math.pi * -1.0
          cv2.line(img2, (w//2,h//2), (w//2+xx, h//2+yy), (255,0,0),2)
          
          if saveImage:
            cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_intersection.jpg", img2)
          cv2.imshow('intersection', img2)
          cv2.waitKey(1)
          #print(x, y, bw, bh, ax, ay)
          ax += conr[0]
          ay += conr[1]
          corner[ii] = (ax, ay)
          spot.ptzSet(ax, ay, z)
          time.sleep(1.0)
          img = spot_webrtc.cvImage.copy()
          if saveImage:
            cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_corner.jpg", img)
          #cv2.imshow('corner', img)
          k = cv2.waitKey(1)

# trace corners via calculated angles
def trace_corners(corners, spot, hfv=63.7, zoom=5.0, saveImage=False):
  k = 0
  hfv2 = hfv / zoom
  count = 0
  spot.ptzSet(corners[0][0][0], corners[0][0][1], zoom)
  time.sleep(1.0)
  for corner in corners:
    time.sleep(2.0)
    for i in range(len(corner)):
      conr = corner[i]
      next = corner[(i+1)%len(corner)]
      ha = next[0]-conr[0]
      va = next[1]-conr[1]
      hn = int (abs(ha)/hfv2*1.25)
      vn = int (abs(va)/(hfv2*9.0/16.0)*1.25)
      if vn > hn:
        hn = vn
      for j in range(hn):
        t = float(j) / float(hn)
        ax = conr[0]*(1.0-t) + next[0] * t
        ay = conr[1]*(1.0-t) + next[1] * t
        spot.ptzSet(ax, ay, zoom)
        time.sleep(2.0)
        img = spot_webrtc.cvImage.copy();
        #img = spot.ptzImage()
        if saveImage:
          timestr = time.strftime('%Y-%m-%d-%H-%M-%S')
          cv2.imwrite('images/tr_' + timestr + '.jpg', img)
        #cv2.imshow('trace', img)
        count += 1
        k = cv2.waitKey(1)

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
        continue
      b = apps2[j]
      if (b['bb'][0] == a['bb'][0] and
          b['bb'][2] == a['bb'][2] and
          b['bb'][1] == a['bb'][1] and
          b['bb'][3] == a['bb'][3] and
          i < j):
        a['included'] = True
      
  for i in range(len(apps2)):
    a = apps2[i]
    if a['included']:
      continue
    for j in range(len(apps2)):
      if i == j:
        continue
      b = apps2[j]
      if (b['bb'][0] <= a['bb'][0] and
          b['bb'][2] >= a['bb'][2] and
          b['bb'][1] <= a['bb'][1] and
          b['bb'][3] >= a['bb'][3]):
        a['included'] = True
        break
  #print(apps)
  appsT = []
  for a in apps2:
    if not a['included']:
      appsT.append(a['app'])
  #print(appsT)
  return(appsT)

def search_rect(img, spot, st1=10, st2=1.5, lt=30, hfv=31.85):
  tanThr = math.tan(85.0/180.0 * math.pi)
  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
  apps = []
  corners = []
  h,w = img.shape[:2]
  wt1 = w / st1
  ht1 = h / st1
  wt2 = int(w / st2)
  ht2 = int(h / st2)
  op = spot.pan
  ot = spot.tilt
  ll = (w // 2) / math.tan(hfv/180.0 * math.pi)

  #cv2.imshow('original', img)

  #
  # color quantization
  #

  # kmean clustering
  K = 4
  Z = img.reshape((-1,3))
  Z = np.float32(Z)
  #ret,label,center = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
  ret,label,center = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_PP_CENTERS)
  center = np.uint8(center)
  res = center[label.flatten()]
  img2 = res.reshape((img.shape))
  if saveImage:
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_KMean.jpg", img2)
  cv2.imshow('kmean', img2)

  # boundary of regions
  cnts = ()
  for i in range(K):
    p1 = center[i] - 1
    p2 = center[i] + 1
    cnts = cnts + find_color(img2, p1, p2, lt, wt1, ht1, wt2, ht2)
    
  imgg0 = np.zeros(shape=(h,w,1), dtype=np.uint8)  
  cv2.drawContours(imgg0, cnts, -1, (255), 1)
  if saveImage:
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_Boundary.jpg", imgg0)
  cv2.imshow('boundary', imgg0)

  # merge boundary contours and remove long and short ones
  cnts, hir = cv2.findContours(imgg0, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  #cnts = tuple(x for x in cnts if chkCntLocation(x, int(w*0.1),int(w*0.9),int(h*0.1),int(h*0.1)))
  cnts = tuple(x for x in cnts if cv2.arcLength(x, True) > lt)
  cnts = tuple(x for x in cnts if chkCntSize(x, wt1, ht1))
  cnts = tuple(x for x in cnts if chkCntSize2(x, wt2, ht2))
  imgg1 = np.zeros(shape=(h,w,1), dtype=np.uint8)  
  cv2.drawContours(imgg1, cnts, -1, (255), 1)
  if saveImage:
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_SelectedContour.jpg", imgg1)
  cv2.imshow('selected contour', imgg1)
      
  #imgg2 = np.zeros(shape=(h,w,1), dtype=np.uint8)  
  imgg2 = img.copy()
  cnts, hirs = cv2.findContours(imgg1, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
  hulls = []
  appsa = []
  for ii in range(len(cnts)):
    cc = cnts[ii]
    hh = hirs[0][ii]
    '''
    if hh[3] != -1: # not outside of the region
      continue
    '''
    hull = cv2.convexHull(cc)
    #print(hull)
    hulls.append(hull)
    rect = cv2.minAreaRect(hull)
    #print(rect[2])
    if abs(rect[2]-90.0) < 10.0:
      o = [1,0,3,2] # portrait or tall
    elif abs(rect[2]) < 10.0:
      o = [2,1,0,3] # landscape or wide
    else:
      continue
    box = cv2.boxPoints(rect)
    box = np.intp(box)
    mpnt = [None, None, None, None]
    mdist = [1.0e+32, 1.0e+32, 1.0e+32, 1.0e+32]
    for hpnt in hull:
      x,y = hpnt[0]
      for j in range(4):
        dx = x - box[j][0]
        dy = y - box[j][1]
        l = dx * dx + dy * dy
        if l < mdist[j]:
          mpnt[j] = hpnt[0]
          mdist[j] = l
      # should be less than 10 pixels from the oriented bounding box
    #print(mdist)
    if (mdist[0] < 100 and mdist[1] < 100 and
        mdist[2] < 100 and mdist[3] < 100):
      app = [[[mpnt[o[0]][0],mpnt[o[0]][1]]],
             [[mpnt[o[1]][0],mpnt[o[1]][1]]],
             [[mpnt[o[2]][0],mpnt[o[2]][1]]],
             [[mpnt[o[3]][0],mpnt[o[3]][1]]]]
      app = np.array(app)
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
      appsa.append(app)
    '''
    # old way
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
    '''
  #print(apps)
  if len(apps) > 1:
    apps = remove_included(apps)
  #print(apps)
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
  imggh = np.zeros(shape=(h,w,1), dtype=np.uint8)
  imgga = np.zeros(shape=(h,w,1), dtype=np.uint8)
  cv2.drawContours(imggh, hulls, -1, (255), 1)
  cv2.drawContours(imgga, appsa, -1, (255), 1)
  cv2.drawContours(imgg2, apps, -1, (0,255,0), 2)
  if saveImage:
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_ConvexHull.jpg", imggh)
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_ApproxRect.jpg", imgga)
    cv2.imwrite('images/'+'{0:06d}'.format(spot_webrtc.frameCount) + "_Rectangle.jpg", imgg2)
  cv2.imshow('convex hull', imggh)
  cv2.imshow('approximated rectangle', imgga)
  cv2.imshow('rectangle', imgg2)
  return corners

def monitor(hostname, robot, process=spot_webrtc.captureT, saveImage=False, traceCorner=True, mdl=None):
  spot_webrtc.frameCount = 0
  spot.set_screen('mech_full')  # PTZ camera
  #spot.set_screen('digi_full')
  #spot.set_screen('pano_full') # for searching window
  #spot.set_screen('c0')
  spot.stand()
  spot.ptzSet(144.0, 12.0, 1.0)
  # Suppress all exceptions and log them instead.
  #sys.stderr = InterceptStdErr()

  if mdl is None:
    mdl = trimesh.load("../cv/cad/sliding_m.obj")
  spot_webrtc.frameCount = 0
  # set up webrtc thread (capture only)
  shutdown_flag = threading.Event()
  webrtc_thread = threading.Thread(
    target=spot_webrtc.start_webrtc, args=[shutdown_flag, hostname, robot.user_token, process],
    daemon=True)

  fc = 0

  #lt = 30
  #st1 = 10
  #st2 = 2
  #hfv = 31.85

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
      spot.ptzSet(144.0, 12.0, 1.0)
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
      corners = search_rect2(img2, mdl, hfv=31.85/spot.zoom, pan=spot.pan, tilt=spot.tilt)
      # end of else
    c = cv2.waitKey(1)
    if c == 27:
      break
    if len(corners) >= 1:
      break
  time.sleep(1.0)
  # fine tune of PAN, TILT value
  improve_corners(corners, spot, [5.0, 10.0])
  #
  if traceCorner:
    trace_corners(corners, spot, zoom=18.0, saveImage=saveImage)
  
      
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
    saveImage = True
    os.makedirs("images", exist_ok=True)
    
  while True:
    if spot.connect():
      spot.set_screen('mech_full')
      #spot.stand()
      # start webrtc monitoring process (just monitoring)
      p = mp.Process(target = spot_webrtc.monitor, args=(spot.hostname, spot.robot))
      p.start()
      monitor(spot.hostname, spot.robot, saveImage=args.save)
      spot.ptzSet(144, 0, 1.0)
      spot.sit()
      spot.disconnect()
        
      # stop webrtc monitoring
      p.terminate()
      p.join()
      break
