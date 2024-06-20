import argparse
import os
import sys
import time
import numpy as np

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.api import trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.util import seconds_to_duration
from bosdyn.client.math_helpers import Quat
from bosdyn.client.spot_cam.media_log import MediaLogClient
from bosdyn.client.spot_check import SpotCheckClient, run_camera_calibration

def hello_spot(config):
    """A simple example of using the Boston Dynamics API to command a Spot robot."""

    # The Boston Dynamics Python library uses Python's logging module to
    # generate output. Applications using the library can specify how
    # the logging information should be output.
    bosdyn.client.util.setup_logging(config.verbose)

    # The SDK object is the primary entry point to the Boston Dynamics API.
    # create_standard_sdk will initialize an SDK object with typical default
    # parameters. The argument passed in is a string identifying the client.
    sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')

    # A Robot object represents a single robot. Clients using the Boston
    # Dynamics API can manage multiple robots, but this tutorial limits
    # access to just one. The network address of the robot needs to be
    # specified to reach it. This can be done with a DNS name
    # (e.g. spot.intranet.example.com) or an IP literal (e.g. 10.0.63.1)
    robot = sdk.create_robot(config.hostname)

    # Clients need to authenticate to a robot before being able to use it.
    bosdyn.client.util.authenticate(robot)

    # Establish time sync with the robot. This kicks off a background thread to establish time sync.
    # Time sync is required to issue commands to the robot. After starting time sync thread, block
    # until sync is established.
    robot.time_sync.wait_for_sync()

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, ' \
                                    'such as the estop SDK example, to configure E-Stop.'

    # The robot state client will allow us to get the robot's state information, and construct
    # a command using frame information published by the robot.
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    # Only one client at a time can operate a robot. Clients acquire a lease to
    # indicate that they want to control a robot. Acquiring may fail if another
    # client is currently controlling the robot. When the client is done
    # controlling the robot, it should return the lease so other clients can
    # control it. The LeaseKeepAlive object takes care of acquiring and returning
    # the lease for us.
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info('Powering on robot... This may take several seconds.')
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Robot power on failed.'
        robot.logger.info('Robot powered on.')

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # RobotCommandBuilder for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        # command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        # print("create camera client")
        # camera_client = robot.ensure_client(MediaLogClient.default_service_name)
        # readParam_from_camera_client(camera_client)

        print("create image client")
        image_client = robot.ensure_client(ImageClient.default_service_name)
        readParam_from_image_client(image_client)

        # # lease_prot = lease_client.acquire()
        # client = robot.ensure_client(SpotCheckClient.default_service_name)
        # # wallet = client.lease_wallet
        # lease = robot.lease_wallet.get_lease()
        # print(lease)
        # # print(wallet)

        # # import pdb;pdb.set_trace()
        
        # run_camera_calibration(client, lease, verbose=True)

def readParam_from_image_client(image_client):
    
    sources = image_client.list_image_sources()
    # only provided with pinhole camera model
    for camera in sources:
        # camera = source.name
        print(camera)
        # if camera.name in ["back_fisheye_image", "frontleft_fisheye_image", "frontright_fisheye_image", "left_fisheye_image", "right_fisheye_image"]:
        #     fx = camera.intrinsics.pinhole.focal_length.x
        #     fy = camera.intrinsics.pinhole.focal_length.y

        #     cx = camera.intrinsics.pinhole.principal_point.x
        #     cy = camera.intrinsics.pinhole.principal_point.y

        #     # intrinsics = camera.intrinsics

        #     print(f"{fx}, {fy}, {cx}, {cy}")
        #     # print(intrinsics)

def readParam_from_camera_client(camera_client):
    cameras = camera_client.list_cameras()
    # cameras = [c for c in cameras if not (c.name == 'pano' or c.name == 'ptz')]

    # for c in cameras:
    #     print(f"camera choices: {c.name}")

    sum_fx = 0
    sum_fy = 0
    sum_cx = 0
    sum_cy = 0
    sum_k1 = 0
    sum_k2 = 0
    sum_k3 = 0
    sum_k4 = 0

    for i in range(len(cameras)):
        camera = cameras[i]
        # print(f"====== camera{i} ======")
        print(f"======== {camera.name} =======")
        print(f"Resolution: width = {camera.resolution.x}, height = {camera.resolution.y}")

        fx = camera.pinhole.focal_length.x
        fy = camera.pinhole.focal_length.y
        cx = camera.pinhole.center_point.x
        cy = camera.pinhole.center_point.y
        print(f"fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}")

        k1 = camera.pinhole.k1
        k2 = camera.pinhole.k2
        k3 = camera.pinhole.k3
        k4 = camera.pinhole.k4
        print(f"k1: {k1}, k2: {k2}, k3: {k3}, k4: {k4}")

        q = Quat(camera.base_tform_sensor.rotation.w, camera.base_tform_sensor.rotation.x,
                    camera.base_tform_sensor.rotation.y, camera.base_tform_sensor.rotation.z)
        R = q.to_matrix()
        T = [
            camera.base_tform_sensor.position.x, camera.base_tform_sensor.position.y,
            camera.base_tform_sensor.position.z
        ]
        print(f"Rotation: {R}\n Translation: {T}")

        sum_fx += fx
        sum_fy += fy
        sum_cx += cx
        sum_cy += cy
        sum_k1 += k1
        sum_k2 += k2
        sum_k3 += k3
        sum_k4 += k4

    avg_fx = sum_fx / 5
    avg_fy = sum_fy / 5
    avg_cx = sum_cx / 5
    avg_cy = sum_cy / 5
    avg_k1 = sum_k1 / 5
    avg_k2 = sum_k2 / 5
    avg_k3 = sum_k3 / 5
    avg_k4 = sum_k4 / 5
    intrinsics = [
        [avg_fx, 0,      avg_cx],
        [0,      avg_fy, avg_cy],
        [0,      0,      1]
    ]
    distortion = [
        [avg_k1, avg_k2, avg_k3, avg_k4]
    ]
    # print("====== Average Intrinsics Matrix ======")
    # for row in intrinsics:
    #     print(row)

    # print("====== Average Distortion Coefficients ======")
    # print(distortion)

def main(argv):
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument(
        '-s', '--save', action='store_true', help=
        'Save the image captured by Spot to the working directory. To chose the save location, use --save_path instead.'
    )
    parser.add_argument(
        '--save-path', default=None, nargs='?', help=
        'Save the image captured by Spot to the provided directory. Invalid path saves to working directory.'
    )
    argv.append("192.168.50.3") # ethernet
    print(argv)
    options = parser.parse_args(argv)
    try:
        hello_spot(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error('Hello, Spot! threw an exception: %r', exc)
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)