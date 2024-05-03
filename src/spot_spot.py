# Copyright (c) 2023 CERLAB, Department of Mechanical Engieering, CMU
#

from __future__ import print_function
import argparse
import sys
import time
import os
import json
import math
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry

from bosdyn.client.image import ImageClient
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit, blocking_selfright
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, get_se2_a_tform_b

from bosdyn import geometry
from bosdyn.api import geometry_pb2, image_pb2, trajectory_pb2, world_object_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
import bosdyn.api.basic_command_pb2 as basic_command_pb2
from CameraService import CameraService
import spot_webrtc
# edit
from bosdyn.client.graph_nav import GraphNavClient
# edit
robot = None
robot_state_client = None
robot_command_client = None

pan = 144.0
tilt = 0.0
zoom = 1.0

hostname = '192.168.80.3'

command_client = None
lease_client = None

def _connect():
    global hostname
    global robot
    global robot_command_client
    global robot_state_client
    global command_client
    global lease_client
    global pan
    global tilt
    global zoom
    
    """A simple example of using the Boston Dynamics API to command a Spot robot."""

    # The Boston Dynamics Python library uses Python's logging module to
    # generate output. Applications using the library can specify how
    # the logging information should be output.
    bosdyn.client.util.setup_logging(False)

    # The SDK object is the primary entry point to the Boston Dynamics API.
    # create_standard_sdk will initialize an SDK object with typical default
    # parameters. The argument passed in is a string identifying the client.
    sdk = bosdyn.client.create_standard_sdk('CerlabSpotSDK')

    # SpotCAM
    bosdyn.client.spot_cam.register_all_service_clients(sdk)
    
    # A Robot object represents a single robot. Clients using the Boston
    # Dynamics API can manage multiple robots, but this tutorial limits
    # access to just one. The network address of the robot needs to be
    # specified to reach it. This can be done with a DNS name
    # (e.g. spot.intranet.example.com) or an IP literal (e.g. 10.0.63.1)
    robot = sdk.create_robot(hostname)

    # Clients need to authenticate to a robot before being able to use it.
    bosdyn.client.util.authenticate(robot)

    # spot Cam
    #sdk2 = bosdyn.client.create_standard_sdk('SPOTCameraSDK')
    #spot_cam.register_all_service_clients(sdk2)
    #cam = sdk2.create_robot("192.168.80.3")
    #robot.authenticate('user', 'scgau6g5w987')
    ptzCam = CameraService(robot)
    # initialize ptz camera position (pan:180, tilt: 0, zoom: 1)
    ptzCam.set_position_ptz(pan, tilt, zoom)
    time.sleep(1)
    
    # Establish time sync with the robot. This kicks off a background thread to establish time sync.
    # Time sync is required to issue commands to the robot. After starting time sync thread, block
    # until sync is established.
    robot.time_sync.wait_for_sync()

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                    "such as the estop SDK example, to configure E-Stop."

    # Only one client at a time can operate a robot. Clients acquire a lease to
    # indicate that they want to control a robot. Acquiring may fail if another
    # client is currently controlling the robot. When the client is done
    # controlling the robot, it should return the lease so other clients can
    # control it. The LeaseKeepAlive object takes care of acquiring and returning
    # the lease for us.

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    # Setup clients for the robot state and robot command services.
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=False):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info("Powering on robot... This may take several seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot powered on.")

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # SpotCommandHelper for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        robot.logger.info("Set robot to accept command...")
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        #blocking_stand(command_client, timeout_sec=10)
        #robot.logger.info("Robot standing.")
        enable_obstacle_avoidance()
        print("current battery charge status: ", battery_status())

def set_default_body_control():
    """Set default body control params to current body position"""
    footprint_R_body = geometry.EulerZXY()
    position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
    rotation = footprint_R_body.to_quaternion()
    pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    traj = trajectory_pb2.SE3Trajectory(points=[point])
    return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
        
def stand():
    global command_client
    try:
        blocking_stand(command_client, timeout_sec=10)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False

def sit():
    global command_client
    try:
        blocking_sit(command_client, timeout_sec=10)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False

def battery_change_pose():
    sit()
    global robot_command_client
    cmd = RobotCommandBuilder.battery_change_pose_command(
        dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT)
    robot_command_client.robot_command(command = cmd,end_time_secs=None)

def self_right():
    global command_client
    try:
        blocking_selfright(command_client, timeout_sec=30)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False

def battery_status():
    return robot_state_client.get_robot_state().battery_states[0].charge_percentage.value
    
def connect():
    try:
        _connect()
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False
    
def _disconnect():
    global robot
    global command_client
    global lease_client

    # Log a comment.
    # Comments logged via this API are written to the robots test log. This is the best way
    # to mark a log as "interesting". These comments will be available to Boston Dynamics
    # devs when diagnosing customer issues.
    log_comment = "CERLAB trial user comment."
    robot.operator_comment(log_comment)
    robot.logger.info('Added comment "%s" to robot log.', log_comment)
    
    #blocking_stand(command_client, timeout_sec=10)
    #blocking_sit(command_client, timeout_sec=10)
    
    # Power the robot off. By specifying "cut_immediately=False", a safe power off command
    # is issued to the robot. This will attempt to sit the robot before powering off.
    robot.power_off(cut_immediately=False, timeout_sec=20)
    assert not robot.is_powered_on(), "Robot power off failed."
    robot.logger.info("Robot safely powered off.")

def disconnect():
    try:
        _disconnect()
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False

def move_rotate(x, y, ang):
    try:
        return relative_move(x, y, ang / 180.0 * math.pi)
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False
    
def rotate(ang):
    try:
        return relative_move(0.0, 0.0, ang / 180.0 * math.pi)
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False

def move(x, y):
    try:
        return relative_move(x, y, 0.0)
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False

def relative_move(dx, dy, dyaw, stairs=False):
    global robot_command_client
    global robot_state_client
    frame_name = ODOM_FRAME_NAME
    
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
    # We do not want to command this goal in body frame because the body will move, thus shifting
    # our goal. Instead, we transform this offset to get the goal position in the output frame
    # (which will be either odom or vision).
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    # Command the robot to go to the goal point in the specified frame. The command will stop at the
    # new position.
    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
        frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
    end_time = 10.0
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=time.time() + end_time)
    # Wait until the robot has reached the goal.
    while True:

        feedback = robot_command_client.robot_command_feedback(cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print("Failed to reach the goal")
            return False
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            print("Arrived at the goal.")
            return True
        time.sleep(1)

    return True

def ptzGet():
    global robot
    global pan
    global tilt
    global zoom
    ptzCam = CameraService(robot)
    js = ptzCam.get_position_ptz()
    #print(js)
    #res = eval(js)
    
def ptzSet(p, t, z):
    global robot
    global pan
    global tilt
    global zoom

    ptzCam = CameraService(robot)
    pan = p
    tilt = t
    zoom = z
    ptzCam.set_position_ptz(pan, tilt, zoom)
    
def ptzTilt(ang):
    global robot
    global pan
    global tilt
    global zoom

    ptzCam = CameraService(robot)
    tilt = ang
    if tilt > 90.0:
        tilt = 90.0
    elif tilt < -30.0:
        tilt = -30.0
    ptzCam.set_position_ptz(pan, tilt, zoom)

def ptzPan(ang):
    global robot
    global pan
    global tilt
    global zoom

    ptzCam = CameraService(robot)
    pan = ang
    ptzCam.set_position_ptz(pan, tilt, zoom)
    
def ptzZoom(val):
    global robot
    global pan
    global tilt
    global zoom

    ptzCam = CameraService(robot)
    zoom = val
    if zoom < 1.0:
        zoom = 1.0
    elif zoom > 30.0:
        zoom = 30.0
    ptzCam.set_position_ptz(pan, tilt, zoom)

def ptzImage():
    global robot
    
    ptzCam = CameraService(robot)
    return ptzCam.cvImage()

def depthData(camera='frontleft'):
    # camera 'frontleft', 'frontright', 'left', 'right', 'back'
    global robot
    
    ptzCam = CameraService(robot)
    return ptzCam.depthData(camera)
    
def setLight(data):
    global robot
    ptzCam = CameraService(robot)
    ptzCam.set_light_brightness(data)

def getLight():
    global robot
    ptzCam = CameraService(robot)
    return ptzCam.get_light_brightness()

def get_screen():
    global robot
    ptzCam = CameraService(robot)
    return ptzCam.get_screen()
    
def set_screen(data):
    global robot
    ptzCam = CameraService(robot)
    return ptzCam.set_screen(data)
    
def list_screens():
    global robot
    ptzCam = CameraService(robot)
    return ptzCam.list_screens()

def monitor_webrtc():
    global hostname, robot
    spot_webrtc.monitor(hostname, robot)

def webrtc_frame_count():
    return spot_sebrtc.frame_count

def disable_obstacle_avoidance():
    obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=True,
                                                disable_vision_foot_obstacle_avoidance=True,
                                                disable_vision_foot_constraint_avoidance=True,
                                                obstacle_avoidance_padding=.001)
    body_control = set_default_body_control()
    mobility_params = spot_command_pb2.MobilityParams(
        obstacle_params=obstacles, body_control=body_control,
        locomotion_hint=spot_command_pb2.HINT_AUTO)

def enable_obstacle_avoidance(distance = 0.3):
    obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=False,
                                                disable_vision_foot_obstacle_avoidance=False,
                                                disable_vision_foot_constraint_avoidance=False,
                                                obstacle_avoidance_padding=distance)
    body_control = set_default_body_control()
    mobility_params = spot_command_pb2.MobilityParams(
        obstacle_params=obstacles, body_control=body_control,
        locomotion_hint=spot_command_pb2.HINT_AUTO)
    
def main(argv):
    global robot
    
    # connect to SPOT
    connect()

    # rotate(45.0)
    # rotate(-45.0)

    # move(0.5, 0.5)
    # move(-0.5, -0.5)

    # ptzCam = CameraService(robot)
    # print(ptzCam.get_position_ptz())

    # ptzCam.set_position_ptz(0,90,1)
    # #print(ptzCam.get_position_ptz())
    # time.sleep(1)
    # ptzCam.take_photo()
    
    # ptzCam.set_position_ptz(120,0,1)
    # time.sleep(1)
    # ptzCam.take_photo()
    # #print(ptzCam.get_position_ptz())

    # end SPOT
    disconnect()

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
