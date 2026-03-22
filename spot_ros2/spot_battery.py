import time
import spot_spot as spot
while True:
    if spot.connect():
        break
spot.battery_change_pose()
