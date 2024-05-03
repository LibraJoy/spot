import spot_spot as spot
import spot_webrtc
from trial import monitor
import cv2

import threading
import time
import multiprocessing as mp
import argparse

parser = argparse.ArgumentParser(description='SPOT Demo2')
parser.add_argument('-s', '--save', action='store_true')
parser.add_argument('-ns', '--no-save', dest = 'save', action = 'store_false')
parser.set_defaults(save=False)
args = parser.parse_args()

print("save     : ", args.save)

while True:
    if spot.connect():
        break

spot.stand()
spot.set_screen('mech_full')
p = mp.Process(target = spot_webrtc.monitor, args=(spot.hostname, spot.robot))
p.start()

time.sleep(5)

spot.move(1,-1)

for c in range(2):
    if not spot.move(1, 0):
        break
    if c == 0:
        monitor(spot.hostname, spot.robot, saveImage=args.save, traceCorner=True)
    else:
        monitor(spot.hostname, spot.robot, saveImage=args.save, traceCorner=False)
    spot.ptzSet(144.0, 0.0, 1.0)
    if not spot.move(1, 0):
        break
    if not spot.rotate(-90):
        break
    if not spot.move(4, 0):
        break
    if not spot.move(0, -2):
        break
    if not spot.rotate(90):
        break
    if not spot.move(0, 4):
        break
spot.move(-1,1)

spot.sit()
spot.disconnect()
p.terminate()
p.join()
