#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import spot_spot as spot
import cv2
import time
import math
import threading
import time
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
import multiprocessing as mp
import bosdyn.client.math_helpers as math_helpers
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, get_se2_a_tform_b
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit, blocking_selfright
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from scipy.spatial.transform import Rotation as R
import numpy as np
robot = None
robot_state_client = None
robot_command_client = None

# For check repeated path_msg
hist_path = None

p = None
webrtc_thread = None
shutdown_flag = None
reached_goal = None

def printlocation():
    global robot_state_client

    curr_transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    vision = curr_transforms.child_to_parent_edge_map.get('vision')
    if vision:
        # Extract the position data (body frame to odom frame)
        position_data = vision.parent_tform_child.position
        

        # Extract the rotation data
        rotation_data = vision.parent_tform_child.rotation
        rot_matrix = R.from_quat([rotation_data.x, rotation_data.y, rotation_data.z, rotation_data.w]).as_matrix() # @ offset_matrix
        # Inverse
        rot_matrix = np.linalg.inv(rot_matrix)
        # Output rotation angle and position
        rot_quaternion = R.from_quat(R.from_matrix(rot_matrix).as_quat())
        rot_euler = rot_quaternion.as_euler('xyz', degrees=True)
        # rot_euler = rot_quaternion.as_euler('xyz', degrees=True)
        pos_xy = np.array([-position_data.x, -position_data.y, 1])
        position = rot_matrix[:2, :]@pos_xy
    # time.sleep(1)
    return position, rot_euler[2]


## 1. make sure the 


def action():
  spot.stand()
  time.sleep(1.0)
  spot.sit()

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
  rospy.loginfo("end spot")


def move(x, y, angle):
    try:
        return global_move(x, y, angle)
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Spot move exception: %r", exc)
        return False

def global_move(dx, dy, dyaw, stairs=False):
    global robot_command_client 
    global robot_state_client
    frame_name = VISION_FRAME_NAME
    
    # heading problem need to be solved, currently no turning head
    transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=0)
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal
    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=dx, goal_y=dy, goal_heading=dyaw,
        frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
    end_time = 10.0
    cmd_id = spot.robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=time.time() + end_time)
    # Wait until the robot has reached the goal.
    while True:

        feedback = spot.robot_command_client.robot_command_feedback(cmd_id)
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

def goal_reached_callback(status):
    # Handle the Bool message indicating goal reached status
    global reached_goal 
    reached_goal = status.data
    # print(reached_goal)

def path_callback(path_msg):
    global reached_goal
    print(reached_goal)
    if not reached_goal:
        # spot.stand()
        x = path_msg.poses[0].pose.position.x
        y = path_msg.poses[0].pose.position.y
        # next_angle = rotate_head(pose[0], pose[1], x, y)
        rospy.loginfo(f"waypoint: x: {x}, y: {y}")
        # rospy.loginfo(f"pose: x: {pose}, angle: {angle}")
        move(x, y,  0)
    if reached_goal:
        print("reached desination!")
        time.sleep(1.0)
        endSpot()

def path_follower():
    # Initializes the ROS node, subscribes to the nav_msgs::Path topic, and starts the ROS loop.
    print("begin")
    rospy.init_node('path_follower', anonymous=True)
    rospy.Subscriber("navigation_SPOT/goal_reached", Bool, goal_reached_callback)
    rospy.Subscriber("navigation_SPOT/waypoints", Path, path_callback)
    rospy.spin()

if __name__ == '__main__':

    while True:
        if spot.connect():
            break

    try:
        path_follower()

    except rospy.ROSInterruptException:
        pass
            