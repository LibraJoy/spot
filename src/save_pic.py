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
parser.set_defaults(save=True)
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

spot.ptzSet(144.0, 0.0, 1.0)
monitor(spot.hostname, spot.robot, saveImage=args.save)

spot.move(0, 0.5)
spot.ptzSet(144.0, 0.0, 1.0)
monitor(spot.hostname, spot.robot, saveImage=args.save)

spot.move(0, -1.0)
spot.ptzSet(144.0, 0.0, 1.0)
monitor(spot.hostname, spot.robot, saveImage=args.save)

spot.move(-0.5, 0.5)
spot.ptzSet(144.0, 0.0, 1.0)
monitor(spot.hostname, spot.robot, saveImage=args.save)

spot.move(1.0, 0.0)
spot.ptzSet(144.0, 0.0, 1.0)
monitor(spot.hostname, spot.robot, saveImage=args.save)

spot.sit()
spot.disconnect()
p.terminate()
p.join()
