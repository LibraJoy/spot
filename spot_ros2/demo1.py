import spot_spot as spot
import spot_webrtc
#from trial import monitor
import cv2
import time

import threading
import time
import multiprocessing as mp

while True:
    if spot.connect():
        break

spot.stand()
spot.set_screen('pano_full')
p = mp.Process(target = spot_webrtc.monitor, args=(spot.hostname, spot.robot))
p.start()

time.sleep(5)

for c in range(3):
    if not spot.move(2, 0):
        break
    if not spot.rotate(-90):
        break
    if not spot.move(4, 0):
        break
    if not spot.rotate(-90):
        break
    if not spot.move(2, 0):
        break
    if not spot.rotate(-90):
        break
    if not spot.move(4, 0):
        break
    if not spot.rotate(-90):
        break
    
spot.sit()
spot.disconnect()
p.terminate()
p.join()
