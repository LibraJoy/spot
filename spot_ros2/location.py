#!/usr/bin/env python
import spot_spot as spot
import spot_webrtc
import cv2
import math
import threading
import time
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
import multiprocessing as mp
import bosdyn.client.math_helpers as math_helpers
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, get_se2_a_tform_b
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit, blocking_selfright
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from scipy.spatial.transform import Rotation as R
import numpy as np
import rospy
from std_msgs.msg import String  # Or another appropriate message type
import json
robot = None
robot_state_client = None
robot_command_client = None
p = None
webrtc_thread = None
shutdown_flag = None

def offset():
    global robot_state_client
    transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    odom = transforms.child_to_parent_edge_map.get('odom')
    offset_matrix = None
    if odom:
      rotation_data = odom.parent_tform_child.rotation
      offset_matrix = np.linalg.inv(R.from_quat([rotation_data.x, rotation_data.y, rotation_data.z, rotation_data.w]).as_matrix())
    return offset_matrix

def printlocation(offset_matrix):
    global robot_state_client

    curr_transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    odom = curr_transforms.child_to_parent_edge_map.get('odom')
    if odom:
        # Extract the position data (body frame to odom frame)
        position_data = odom.parent_tform_child.position
        pos_xy = np.array([-position_data.x, -position_data.y, 1])

        # Extract the rotation data
        rotation_data = odom.parent_tform_child.rotation
        rot_matrix = R.from_quat([rotation_data.x, rotation_data.y, rotation_data.z, rotation_data.w]).as_matrix() @ offset_matrix
        
        # Inverse
        rot_matrix = np.linalg.inv(rot_matrix)

        # Print rotation angle and position
        rot_quaternion = R.from_quat(R.from_matrix(rot_matrix).as_quat())
        rot_euler = rot_quaternion.as_euler('xyz', degrees=True)
        print(f"Rotation Matrix {rot_quaternion} \n")

        new_p = rot_matrix[:2, :]@pos_xy
        print(f"Position X: {new_p[0]} m, Position Y: {new_p[1]} m\n")

    time.sleep(1)
    
    return True

if __name__ == '__main__':
  while True:
    if spot.connect():
      break
    offset_matrix = offset()
    printlocation(offset_matrix)
