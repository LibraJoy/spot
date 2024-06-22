#!/usr/bin/env python3
import spot.spot_spot as spot
import time
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
robot = None
robot_state_client = None
robot_command_client = None
p = None
webrtc_thread = None
shutdown_flag = None


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

        # rot_euler = rot_quaternion.as_euler('xyz', degrees=True)
        pos_xy = np.array([-position_data.x, -position_data.y, 1])
        position = rot_matrix[:2, :]@pos_xy
        pos_z = position_data.z
    # time.sleep(1)
    return position, rot_quaternion,pos_z



if __name__ == '__main__':
    try:
        pose_publisher = rospy.Publisher('/spot/pose', PoseStamped, queue_size=10)
        odom_publisher = rospy.Publisher('/spot/odom', Odometry, queue_size=10)
        rospy.init_node('spot_publisher', anonymous=True)
        rate = rospy.Rate(30)
        # print("Starting Spot Publisher")
        if spot.connect():
            # offset_matrix = offset()
            print("success connected")
            while not rospy.is_shutdown():
                
                    
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.header.frame_id = "map"

                    odom_msg = Odometry()
                    odom_msg.header.stamp = rospy.Time.now()
                    odom_msg.header.frame_id = "map"

                    position, quaternion,pos_z = printlocation()
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

                    rospy.loginfo(pose_msg)
                    pose_publisher.publish(pose_msg)

                    rospy.loginfo(odom_msg)
                    odom_publisher.publish(odom_msg)
                    rate.sleep()
        else:
            print("connection fails")

    except rospy.ROSInterruptException:
        pass