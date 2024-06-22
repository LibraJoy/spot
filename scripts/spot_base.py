#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import spot.spot_spot as spot

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
from nav_msgs.msg import Odometry
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


class spotMoveBase:
    def __init__(self):
        # pass
        self.rate = rospy.Rate(15)
        rospy.Subscriber("/spot/waypoint", PoseStamped, self.goal_pose_sub_callback)
        self.pose_publisher = rospy.Publisher('/spot/pose', PoseStamped, queue_size=10)
        self.odom_publisher = rospy.Publisher('/spot/odom', Odometry, queue_size=10)
        rospy.Timer(rospy.Duration(0.03), self.odom_pub_timer_callback)
        rospy.Timer(rospy.Duration(0.05), self.move_status_check_timer_callback)
        self.goal = [0., 0. ,0.] # x,y,yaw
        self.cmd_id = None

    def odom_pub_timer_callback(self, event):
        start_time = rospy.Time.now()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"

        position, quaternion,pos_z = self.get_location()
        qx, qy, qz, qw = quaternion.as_quat()
        # Set position
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = -pos_z

        # Set orientation
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        # Set odom pos
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = -pos_z

        # Set odom orientation
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # rospy.loginfo(pose_msg)
        self.pose_publisher.publish(pose_msg)

        # rospy.loginfo(odom_msg)
        self.odom_publisher.publish(odom_msg)
        print(f"odom pub time: {rospy.Time.now() - start_time}")

    def goal_pose_sub_callback(self, msg):
        if self.goal[0] == msg.pose.position.x and self.goal[1] == msg.pose.position.y:
            rospy.loginfo("subscribed goal has not changed, do not upddate command")
            return

        self.goal = [msg.pose.position.x, msg.pose.position.y, 0]
        rospy.loginfo(f"waypoint: x: {self.goal[0]}, y: {self.goal[1]}")

        global robot_command_client 
        global robot_state_client
        frame_name = VISION_FRAME_NAME
        [dx, dy, dyaw] = self.goal
        
        # heading problem need to be solved, currently no turning head
        transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
        body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=0)
        out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
        out_tform_goal = out_tform_body * body_tform_goal
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=dx, goal_y=dy, goal_heading=dyaw,
            frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=False))
        end_time = 10.0
        self.cmd_id = spot.robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                    end_time_secs=time.time() + end_time)
        print(f"movement command request sent: {self.goal}")

    def move_status_check_timer_callback(self, event):
        if not self.cmd_id:
            rospy.loginfo("no command id, skip move status check")
            return
        print(f"current goal in global move: {self.goal}")
        print(f"current position: {self.get_location()}")
        feedback = spot.robot_command_client.robot_command_feedback(self.cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print("Failed to reach the goal")
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            print("Arrived at the goal.")

    # def get_location(self):
    #     global robot_state_client

    #     curr_transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    #     vision = curr_transforms.child_to_parent_edge_map.get('vision')
    #     if vision:
    #         # Extract the position data (body frame to odom frame)
    #         position_data = vision.parent_tform_child.position
            

    #         # Extract the rotation data
    #         rotation_data = vision.parent_tform_child.rotation
    #         rot_matrix = R.from_quat([rotation_data.x, rotation_data.y, rotation_data.z, rotation_data.w]).as_matrix() # @ offset_matrix
    #         # Inverse
    #         rot_matrix = np.linalg.inv(rot_matrix)
    #         # Output rotation angle and position
    #         rot_quaternion = R.from_quat(R.from_matrix(rot_matrix).as_quat())
    #         rot_euler = rot_quaternion.as_euler('xyz', degrees=True)
    #         # rot_euler = rot_quaternion.as_euler('xyz', degrees=True)
    #         pos_xy = np.array([-position_data.x, -position_data.y, 1])
    #         position = rot_matrix[:2, :]@pos_xy
    #     # time.sleep(1)
    #     return position, rot_euler[2]

    def get_location(self):
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

            # rot_euler = rot_quaternion.as_euler('xyz', degrees=True)
            pos_xy = np.array([-position_data.x, -position_data.y, 1])
            position = rot_matrix[:2, :]@pos_xy
            pos_z = position_data.z
        # time.sleep(1)
        return position, rot_quaternion, pos_z


    ## 1. make sure the 


    def action(self):
        spot.stand()
        time.sleep(1.0)
        spot.sit()

    def endSpot(self):
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


    def move(self):
        try:
            return self.global_move()
        except Exception as exc:  # pylint: disable=broad-except
            logger = bosdyn.client.util.get_logger()
            logger.error("Spot move exception: %r", exc)
            return False

    # def global_move(self, stairs=False):
    #     global robot_command_client 
    #     global robot_state_client
    #     frame_name = VISION_FRAME_NAME
    #     [dx, dy, dyaw] = self.goal
        
    #     # heading problem need to be solved, currently no turning head
    #     transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    #     body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=0)
    #     out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    #     out_tform_goal = out_tform_body * body_tform_goal
    #     robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
    #         goal_x=dx, goal_y=dy, goal_heading=dyaw,
    #         frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
    #     end_time = 10.0
    #     cmd_id = spot.robot_command_client.robot_command(lease=None, command=robot_cmd,
    #                                                 end_time_secs=time.time() + end_time)
    #     # Wait until the robot has reached the goal.
        
    #     while dx==self.goal[0] and dy==self.goal[1] and dyaw==self.goal[2]:
    #         print(f"goal in global move: {self.goal}")
    #         print(f"dx, dy , dyaw in global move: {dx}, {dy}, {dyaw}")
    #         print(f"current position: {self.get_location()}")
    #         feedback = spot.robot_command_client.robot_command_feedback(cmd_id)
    #         mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
    #         if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
    #             print("Failed to reach the goal")
    #             return False
    #         traj_feedback = mobility_feedback.se2_trajectory_feedback
    #         if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
    #                 traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
    #             print("Arrived at the goal.")
    #             return True
    #         self.rate.sleep()
            

    #     rospy.logwarn("goal has changed. exit spot local planner loop!")
    #     return True

    def goal_reached_callback(self, status):
        pass

    def path_callback(self, path_msg):
        self.move()

if __name__ == '__main__':
    while True:
        if spot.connect():
            break
        else:
            print("connection fails")
    
    print("begin")
    rospy.init_node('spot_base', anonymous=True)
    
    spot_move_base = spotMoveBase()
    
    rospy.spin()