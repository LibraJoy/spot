import spot_spot as spot
import spot_webrtc
import cv2
import time

import threading
import time
import multiprocessing as mp

p = None
webrtc_thread = None
shutdown_flag = None
ang = 15.0
dist = 1.0

# simple motion
def action():
  spot.stand()
  time.sleep(1.0)
  spot.sit()
  
def startMonitor(hostname, robot, process=spot_webrtc.captureT, movement = action):
  global webrtc_thread
  global shutdown_flag
  spot_webrtc.frameCount = 0
  spot.set_screen('mech_full')  # PTZ camera
  #spot.set_screen('digi_full')
  #spot.set_screen('pano_full') # for searching window
  #spot.set_screen('c0')
  spot.stand()
  # Suppress all exceptions and log them instead.
  #sys.stderr = InterceptStdErr()

  spot_webrtc.frameCount = 0
  # set up webrtc thread (capture only)
  if webrtc_thread is None:
    shutdown_flag = threading.Event()
    webrtc_thread = threading.Thread(
      target=spot_webrtc.start_webrtc, args=[shutdown_flag, hostname, robot.user_token, process],
      daemon=True)

  fc = 0

  corners = []
  # start webrtc thread
  webrtc_thread.start()
  done = False
  while True:
    #print(spot_webrtc.frameCount)
    if not webrtc_thread.is_alive():
      break
    elif spot_webrtc.frameCount == 0:
      time.sleep(0.1)
      tm1 = time.time()
    else:
      movement()
      done = True
      # end of else
    c = cv2.waitKey(1)
    if c == 27 or done:
      break

def endSpot():
  global p
  global webrtc_thread
  if webrtc_thread is not None:
    # stop webrtc capture thread        
    shutdown_flag.set()
    try:
      webrtc_thread.join()
      #print('Successfully saved webrtc images to local directory.')
    except KeyboardInterrupt:
      shutdown_flag.set()
      webrtc_thread.join(timeout=3.0)

  time.sleep(1.0)      
  spot.sit()

  cv2.destroyAllWindows()
  # close webRTC process
  if p is not None:
    p.terminate()
    p.join()

# main loop    
def moveSpot(movement=action):
  global p
  while True:
    if spot.connect():
      break
  spot.stand()
  #spot.set_screen('pano_full')
  spot.set_screen('mech_full')

  # webRTC showing (different process)
  p = mp.Process(target = spot_webrtc.monitor, args=(spot.hostname, spot.robot))
  p.start()

  startMonitor(spot.hostname, spot.robot, movement=movement)


if __name__ == '__main__':
  moveSpot(movement=action)
  time.sleep(1.0)
  endSpot()
