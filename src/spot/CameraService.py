import tempfile
import shutil
import os
import shutil
import tempfile
import cv2 
import numpy as np

from bosdyn.client.spot_cam.ptz import PtzClient
from bosdyn.client.spot_cam.media_log import MediaLogClient
from bosdyn.client.robot import Robot
from bosdyn.api.spot_cam import ptz_pb2
from bosdyn.api import image_pb2
from bosdyn.api.spot_cam import logging_pb2, camera_pb2, compositor_pb2, LED_pb2
#from bosdyn.client.command_line import (Command, Subcommands)
from bosdyn.client.spot_cam.lighting import LightingClient
from bosdyn.client.spot_cam.compositor import CompositorClient

# depth camera
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient
 
from datetime import datetime
 
from PIL import Image
 
class CameraService():
 
    def __init__(self,robot = Robot()):
 
        """
    Description:
 
        This is a service that helps to control the SPOT PTZ camera more easily.
 
        To use this service it is necessary to first register the spot_cam (from bosdyn.client) services in the sdk instance.
        spot_cam.register_all_service_clients(sdk).
 
    Parameters:
 
        robot (bosdyn.client.robot) = An instance of the robot that we want to control its camera.
 
    Example:
 
        sdk = bosdyn.client.create_standard_sdk("SPOTCameraSDK")
 
        spot_cam.register_all_service_clients(sdk)
 
        spot = sdk.create_robot(spot_config.robot_ip)
 
        spot.authenticate(spot_config.user, spot_config.password)
 
        ptzCam = CameraService(spot)
        """
 
        self.robot = robot
        self.ptz_description = ptz_pb2.PtzDescription(name = 'mech')
        self.cam_client= self.robot.ensure_client(PtzClient.default_service_name)
        self.media_client = self.robot.ensure_client(MediaLogClient.default_service_name)
        self.led_client = self.robot.ensure_client(LightingClient.default_service_name)
        self.composer_client = self.robot.ensure_client(CompositorClient.default_service_name)
        # depth image (realsense)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)

    def get_position_ptz(self):
        """
    Description:
 
        returns a dictionary with the current position (pan, tilt, zoom) of the PTZ camera.
 
    Example:
 
        ptzCam = CameraService(spot)
 
        currentCamZoom = ptzCam.get_position_ptz["zoom"]
        """
        position = self.cam_client.get_ptz_position(self.ptz_description)
        return position
 
    def set_position_ptz(self,pan,tilt,zoom):
        """
    Description:
 
        Change the position of the PTZ camera.
 
    Parameters:
 
        pan (number)
 
        tilt (number)
 
        zoom (number)
 
    Example:
 
        ptzCam = CameraService(spot)
 
        ptzCam.set_position_ptz(0,90,1)
        """
        new_position = self.cam_client.set_ptz_position(self.ptz_description, pan, tilt, zoom)
        return new_position
    
    def take_photo(self):
        """
    Description:
 
        It takes a photo from the PTZ camera and stores it in the directory "./img/ptz/" as jpg with the name of the time, day and year it was taken.
    Example:
 
        ptzCam = CameraService(spot)
 
        ptzCam.take_photo()
 
        """
        args = (camera_pb2.Camera(name="ptz"), logging_pb2.Logpoint.STILLIMAGE)
        lp = self.media_client.store(*args)
 
        while lp.status != logging_pb2.Logpoint.COMPLETE:
            lp = self.media_client.get_status(lp)
        lp, img = self.robot.ensure_client(MediaLogClient.default_service_name).retrieve(lp)
 
        with tempfile.NamedTemporaryFile(delete=False) as img_file:
                img_file.write(img)
                src_filename = img_file.name
 
        dst_filename = os.path.basename(src_filename)
        shutil.move(src_filename, 'tempFile.rgb24')
 
        with open('tempFile.rgb24', mode='rb') as fd:
            data = fd.read()
 
        mode = 'RGB'
        dt_str = datetime.now().strftime("-%d-%m-%Y-%H:%M:%S")
        image = Image.frombuffer(mode, (lp.image_params.width, lp.image_params.height), data, 'raw', mode, 0, 1)
        image.save('./img/ptz/photo-at-{}.jpg'.format(dt_str))
        os.remove('tempFile.rgb24')
        return True

    def pilImage(self):
        args = (camera_pb2.Camera(name="ptz"), logging_pb2.Logpoint.STILLIMAGE)
        lp = self.media_client.store(*args)
 
        while lp.status != logging_pb2.Logpoint.COMPLETE:
            lp = self.media_client.get_status(lp)
        lp, img = self.robot.ensure_client(MediaLogClient.default_service_name).retrieve(lp)
 
        with tempfile.NamedTemporaryFile(delete=False) as img_file:
            img_file.write(img)
            src_filename = img_file.name
                
        with open(src_filename, mode='rb') as fd:
            data = fd.read()
        mode = 'RGB'
        image = Image.frombuffer(mode, (lp.image_params.width, lp.image_params.height), data, 'raw', mode, 0, 1)
        return image
        
    def cvImage(self):
        cvImg = cv2.cvtColor(np.array(self.pilImage()), cv2.COLOR_RGB2BGR)
        return cvImg
        
    def raw_photo(self):
        """
    Description:
 
        It takes a photo from the PTZ camera and stores it in the directory "./img/temp/" as rgb24 with the name of the time, day and year it was taken.
    Example:
 
        ptzCam = CameraService(spot)
 
        ptzCam.raw_photo()
 
        """
        args = (camera_pb2.Camera(name="ptz"), logging_pb2.Logpoint.STILLIMAGE)
        lp = self.media_client.store(*args)
 
        while lp.status != logging_pb2.Logpoint.COMPLETE:
            lp = self.media_client.get_status(lp)
        lp, img = self.robot.ensure_client(MediaLogClient.default_service_name).retrieve(lp)
 
        with tempfile.NamedTemporaryFile(delete=False) as img_file:
                img_file.write(img)
                src_filename = img_file.name
 
        dt_str = datetime.now().strftime("-%d-%m-%Y-%H:%M:%S")
        shutil.move(src_filename, './img/temp/photo-at-{}.rgb24'.format(dt_str))

    def get_light_brightness(self):
        bright = self.led_client.get_led_brightness()
        return bright

    def set_light_brightness(self, data):
        self.led_client.set_led_brightness(data)

    # default: 'mech'
    def get_screen(self):
        return self.composer_client.get_screen()

    def list_screens(self):
        return self.composer_client.list_screens()
    
    def set_screen(self, data):
        self.composer_client.set_screen(data)

    def get_focus(self):
        return self.cam_client.get_ptz_focus_state()

    def set_focus_auto(self):
        self.cam_client.set_ptz_focus_state(ptz_pb2.PtzFocusState.PTZ_FOCUS_AUTO, None, None)

    def set_focus_manual(self, distance):
        self.cam_client.set_ptz_focus_state(ptz_pb2.PtzFocusState.PTZ_FOCUS_MANUAL, distance, None)

    def depthData(self, camera='frontleft'):
        # camera 'frontleft', 'frontright', 'left', 'right', 'back'
        #sources = [camera + '_depth', camera + '_visual_in_depth_frame']
        sources = [camera + '_depth_in_visual_frame', camera + '_fisheye_image']
        #print(sources)
        image_responses = self.image_client.get_image_from_sources(sources)
        if len(image_responses) < 2:
            print('Error: failed to get depth images.')
            return False
        cv_depth = np.frombuffer(image_responses[0].shot.image.data, dtype=np.uint16)
        cv_depth = cv_depth.reshape(image_responses[0].shot.image.rows,
                                    image_responses[0].shot.image.cols)
        cv_visual = cv2.imdecode(np.frombuffer(image_responses[1].shot.image.data, dtype=np.uint8), -1)
        if image_responses[0].source.name[0:5] == 'front':
            cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_90_CLOCKWISE)
            cv_visual = cv2.rotate(cv_visual, cv2.ROTATE_90_CLOCKWISE)

        elif image_responses[0].source.name[0:5] == 'right':
            cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_180)
            cv_visual = cv2.rotate(cv_visual, cv2.ROTATE_180)
        return cv_visual,cv_depth