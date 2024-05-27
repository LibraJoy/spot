import spot_spot as spot
import spot_webrtc
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

p = None
webrtc_thread = None
shutdown_flag = None


# simple motion
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

  cv2.destroyAllWindows()
  # close webRTC process
  if p is not None:
    p.terminate()
    p.join()

def rotate(ang, offset_matrix):
    try:
        return relative_move(0.0, 0.0, ang / 180.0 * math.pi, offset_matrix)
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False
    
def move(x, y, offset_matrix):
    try:
        return relative_move(x, y, 0.0, offset_matrix)
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False

def offset():
    global robot_state_client
    transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    odom = transforms.child_to_parent_edge_map.get('odom')
    offset_matrix = None
    if odom:
      rotation_data = odom.parent_tform_child.rotation
      offset_matrix = np.linalg.inv(R.from_quat([rotation_data.x, rotation_data.y, rotation_data.z, rotation_data.w]).as_matrix())
    return offset_matrix

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

# main loop    
def moveSpot(movement=action):
    global p
    walk_dist = 0
    while True:
        if spot.connect():
            break
    spot.stand()
    offset_matrix = offset()
    move(2, 0, offset_matrix)
    rotate(90, offset_matrix)
    # rotate(-90, offset_matrix)
    move(2, 0, offset_matrix)
    rotate(90, offset_matrix)
    move(2, 0, offset_matrix)
    rotate(90, offset_matrix)    
    move(2, 0, offset_matrix)
    rotate(90, offset_matrix)
  #spot.set_screen('pano_full')

  

    # webRTC showing (different process)
#     graph_nav_client = spot.robot.ensure_client(GraphNavClient.default_service_name)
#     state = graph_nav_client.get_localization_state()
#     current_location = state.localization.seed_tform_body
#     print(f'Got localization_v1: \n{current_location}')
#     if math_helpers.SE3Pose.from_proto(current_location) == None:
#      print("empty")
#     else:
#      print(f'Got localization_v2: \n{math_helpers.SE3Pose.from_proto(current_location)}')
#   p = mp.Process(target = spot_webrtc.monitor, args=(spot.hostname, spot.robot))
#   p.start()
#   startMonitor(spot.hostname, spot.robot, movement=movement)

if __name__ == '__main__':
  moveSpot(movement=action)
  time.sleep(1.0)
  endSpot()

# rot_mat [[ 0.21278759  0.9770976   0.00131151]
#  [-0.97709729  0.21278505  0.00184496]
#  [ 0.00152363 -0.00167406  0.99999744]] 

#rot_mat [[-8.83809657e-01  4.67846000e-01  7.81450615e-04]
#  [-4.67827277e-01 -8.83787022e-01  7.62490938e-03]
#  [ 4.25791927e-03  6.37338463e-03  9.99970625e-01]] 
