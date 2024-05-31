# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

import asyncio
import base64
import json
import logging
import sys
import threading
import time
import os

import cv2
import numpy as np
import requests
from aiortc import MediaStreamTrack, RTCConfiguration, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRecorder
from spot.webrtc_client import WebRTCClient

import bosdyn.client

from bosdyn.client import spot_cam
from bosdyn.client.util import add_base_arguments, setup_logging

logging.basicConfig(level=logging.DEBUG, filename='webrtc.log', filemode='a+')
STDERR = logging.getLogger('stderr')

frameCount = 0
cvImage = None
frameR = None
rgbImage = None

class InterceptStdErr:
    """Intercept all exceptions and print them to StdErr without interrupting."""
    _stderr = sys.stderr

    def __init__(self):
        pass

    def write(self, data):
        pass
        #STDERR.error(data)


# WebRTC must be in its own thread with its own event loop.
def start_webrtc(shutdown_flag, hostname, token, process_func, recorder=None):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    config = RTCConfiguration(iceServers=[])
    #print('token', token)
    client = WebRTCClient(hostname, 31102, 'h264.sdp', False,
                          token, config, media_recorder=recorder)

    asyncio.gather(client.start(), process_func(client, shutdown_flag),
                   monitor_shutdown(shutdown_flag, client))
    loop.run_forever()

# Frame processing occurs; otherwise it waits.
async def monitorT(client, shutdown_flag, save=False):
    global frameCount
    global cvImage
    # image file save
    if save:
        try:
            os.makedirs("images")
        except FileExistsError:
            pass
    w_name = 'WebRTC Monitor'
    #cv2.namedWindow(w_name, cv2.WINDOW_NORMAL)
    frameCount = 0
    while asyncio.get_event_loop().is_running():
        #print(count)
        try:
            frame = await client.video_frame_queue.get()

            pil_image = frame.to_image()
            mat = np.array(pil_image)
            # OpenCV needs BGR
            img = cv2.cvtColor(mat, cv2.COLOR_RGB2BGR)
            cv2.imshow(w_name, img)
            if save:
                cv2.imwrite('images/'+'{0:06d}'.format(frameCount)+'.jpg', img)
            frameCount += 1
            c = cv2.waitKey(1)
            if c == 27:
                break
        except Exception as e:
            pass
            #print(e)
        try:
            # discard audio frames
            while not client.audio_frame_queue.empty():
                await client.audio_frame_queue.get()
        except Exception as e:
            pass
            #print(e)
    shutdown_flag.set()

# Frame processing occurs; otherwise it waits.
async def captureT(client, shutdown_flag):
    global frameCount
    global cvImage
    global frameR
    global rgbImage
    frameCount = 0
    frameR = None
    while asyncio.get_event_loop().is_running():
        #print(count)
        try:
            frame = await client.video_frame_queue.get()
            frameR = frame
            # frameCount += 1
            pil_image = frame.to_image()
            mat = np.array(pil_image)
            rgbImage = mat.copy()
            # OpenCV needs BGR
            cvImage = cv2.cvtColor(mat, cv2.COLOR_RGB2BGR)
            #cv2.imshow('display', cvImage)
            #c = cv2.waitKey(1)
            #if c == 27:
            #    break
            frameCount += 1
        except Exception as e:
            pass
            #print(e)
        try:
            # discard audio frames
            while not client.audio_frame_queue.empty():
                await client.audio_frame_queue.get()
        except Exception as e:
            pass
            #print(e)
    shutdown_flag.set()

# Flag must be monitored in a different coroutine and sleep to allow frame
# processing to occur.
async def monitor_shutdown(shutdown_flag, client):
    global frameCount
    while not shutdown_flag.is_set():
        await asyncio.sleep(1.0)

    await client.pc.close()
    asyncio.get_event_loop().stop()
    frameCount = 0

def monitor(hostname, robot, process=monitorT):
    global frameCount
    frameCount = 0
    #sdk = bosdyn.client.create_standard_sdk('Spot CAM Client')
    #spot_cam.register_all_service_clients(sdk)

    # Suppress all exceptions and log them instead.
    #sys.stderr = InterceptStdErr()

    shutdown_flag = threading.Event()
    webrtc_thread = threading.Thread(
        target=start_webrtc, args=[shutdown_flag, hostname, robot.user_token, process],
        daemon=True)
    webrtc_thread.start()

    try:
        webrtc_thread.join()
        #print('Successfully saved webrtc images to local directory.')
    except KeyboardInterrupt:
        shutdown_flag.set()
        webrtc_thread.join(timeout=3.0)

