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
def relative_move(dx, dy, dyaw, offset_matrix, stairs=False):
    global robot_command_client 
    global robot_state_client
    frame_name = VISION_FRAME_NAME

    transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

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
        goal_x=dx, goal_y=dy, goal_heading=out_tform_goal.angle,
        frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
    end_time = 10.0
    cmd_id = spot.robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=time.time() + end_time)
    # Wait until the robot has reached the goal.
    # odom = transforms.child_to_parent_edge_map.get('odom')
    # if odom:
    #   rotation_data = odom.parent_tform_child.rotation
    #   offset_matrix = np.linalg.inv(R.from_quat([rotation_data.x, rotation_data.y, rotation_data.z, rotation_data.w]).as_matrix())
    while True:


        curr_transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
        curr_out_tform_body = get_se2_a_tform_b(curr_transforms, frame_name, BODY_FRAME_NAME)
        odom = curr_transforms.child_to_parent_edge_map.get('odom')
        if odom:
            # Extract the position data (body frame to odom frame)
            position_data = odom.parent_tform_child.position
            
            # Extract the rotation data
            rotation_data = odom.parent_tform_child.rotation
            rot_matrix = R.from_quat([rotation_data.x, rotation_data.y, rotation_data.z, rotation_data.w]).as_matrix() @ offset_matrix
            
            # inverse
            rot_matrix = np.linalg.inv(rot_matrix)
            
            print(rot_matrix)
            rot_quaternion = R.from_quat(R.from_matrix(rot_matrix).as_quat())
            rot_euler = rot_quaternion.as_euler('xyz', degrees=True)
            print(f"rot_euler {rot_euler} \n")

            # make position coordinate vector p(x,y,1), p = R@p -> new p is new position
            pos_xy = np.array([-position_data.x, -position_data.y, 1])
            new_p = rot_matrix[:2, :]@pos_xy
            print(f"position {new_p} \n")
            offset_quaternion = R.from_quat(R.from_matrix(offset_matrix).as_quat())
            offset_euler = offset_quaternion.as_euler('xyz', degrees=True)
            # print(f"offset_euler {offset_euler}")
        
    


        feedback = spot.robot_command_client.robot_command_feedback(cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print("Failed to reach the goal")
            return False
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            print("Arrived at the goal.")
            offset_quaternion = R.from_quat(R.from_matrix(offset_matrix).as_quat())
            offset_euler = offset_quaternion.as_euler('xyz', degrees=True)
            # print(offset_euler)
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
    # move(-0.5, 0, offset_matrix)
    # move(-1, 0, offset_matrix)
    # rotate(-90, offset_matrix)
    move(0, 0, offset_matrix)
    # move(-0.2327, -1.2186, offset_matrix)

if __name__ == '__main__':
  moveSpot(movement=action)
  time.sleep(1.0)
  endSpot()
