import spot_spot as spot
import spot_webrtc
from trial_2win import monitor
import multiprocessing as mp
import time

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
spot.set_screen('mech_full')
spot.stand()
spot.move(0, -1.0)
time.sleep(5.0)
p = mp.Process(target = spot_webrtc.monitor, args=(spot.hostname, spot.robot))
p.start()

monitor(spot.hostname, spot.robot,saveImage=args.save, maxCount=2)

spot.move(0, 1.0)
spot.sit()
spot.disconnect()
p.terminate()
p.join()
