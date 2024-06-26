#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
from spot.msg import SemanticLabel
from geometry_msgs.msg import PoseStamped
import spot.spot_spot as spot
import spot.spot_move as spot_move
import spot.spot_webrtc as spot_webrtc

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
from bosdyn.client.frame_helpers import get_vision_tform_body
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, get_se2_a_tform_b
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit, blocking_selfright
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from scipy.spatial.transform import Rotation as R
import numpy as np
import torch
import yolov7
import os
import random
import threading
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

        # ROS
        rospy.Subscriber("/spot/waypoint", PoseStamped, self.goal_pose_sub_callback)
        self.pose_pub = rospy.Publisher('/spot/pose', PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/spot/odom', Odometry, queue_size=10)
        self.img_pub = rospy.Publisher('/spot_image', Image, queue_size=10)
        self.yolo_img_pub = rospy.Publisher('yolo_image', Image, queue_size=10)
        self.bbox_pub = rospy.Publisher('yolo_bbox', Detection2DArray, queue_size=10)
        self.label_pub = rospy.Publisher('yolo_label', SemanticLabel, queue_size=10)
        rospy.Timer(rospy.Duration(0.03), self.odom_pub_timer_callback)
        rospy.Timer(rospy.Duration(0.05), self.move_status_check_timer_callback)
        rospy.Timer(rospy.Duration(0.1), self.raw_img_callback)


        self.goal = [0., 0. ,0.] # x,y,yaw
        self.cmd_id = None
        self.img = None
        self.img_received = False

        # # image service
        self.bridge = CvBridge()
        spot.set_screen('pano_full')
        self.startMonitor(spot.hostname, spot.robot)
        # spot_move.startMonitor(spot.hostname, spot.robot, movement=self.image_getter)
        # creat image getter thread
        # self.img_getter_thread = threading.Thread(target=self.image_getter)
        # self.img_getter_thread.start()

        # # yolo
        # rospack = rospkg.RosPack()
        # pkg_path = rospack.get_path('spot')
        # # model_path = os.path.join(pkg_path, 'src/spot', 'yolov7.pt')
        # model_path = os.path.join(pkg_path, 'src/spot', 'yolov7-tiny.pt')
        # device = 'cpu'
        # if torch.cuda.is_available():
        #     device = 'cuda:0'
        # torch.device(device)
        # # model = yolov7.load('yolov7.pt') # ... COCO
        # self.model = yolov7.load(model_path)

    def startMonitor(self, hostname, robot, process=spot_webrtc.captureT):
        global webrtc_thread
        global shutdown_flag
        spot_webrtc.frameCount = 0
        spot_webrtc.frameR = None
        # spot.set_screen('mech_full')  # PTZ camera
        #spot.set_screen('digi_full')
        spot.set_screen('pano_full') # for searching window
        #spot.set_screen('c0')
        #   spot.stand()
        # Suppress all exceptions and log them instead.
        #sys.stderr = InterceptStdErr()

        spot_webrtc.frameCount = 0
        spot_webrtc.frameR = None
        # set up webrtc thread (capture only)
        if webrtc_thread is None:
            shutdown_flag = threading.Event()
            webrtc_thread = threading.Thread(
            target=spot_webrtc.start_webrtc, args=[shutdown_flag, hostname, robot.user_token, process],
            daemon=True)

        # start webrtc thread
        webrtc_thread.start()

        # rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            c = cv2.waitKey(1)
            if c == 27:
                break
            if not webrtc_thread.is_alive():
                break
            elif spot_webrtc.frameCount == 0:
                tm1 = time.time()
                print("-------------------------- frame count = 0 ----------------------")
                if spot_webrtc.frameR is None:
                    print("-------------------- NO FRAME RECEIVED FROM QUEUE --------------------")
            else:
                print("-----------------------IMAGE QUEUE READY-----------------------")
                self.img_received = True
                break


    def image_getter(self):
        while not rospy.is_shutdown():
            # print("image getter")
            self.rgb = spot_webrtc.rgbImage.copy()
            self.img = spot_webrtc.cvImage.copy()


        # results = self.model(rgb)
        # p = results.pred[0]
        # box = p[:,:4] # bbox start and end points
        # conf = p[:,4] # condidence
        # cat = p[:,5] # category id
        # tl = 3
        # for i in range(len(box)):
        #     label = '{} {:.1f}%'.format(self.model.names[int(cat[i])], conf[i]*100.0)
        #     # print(label)
        #     color = [random.randint(0, 255) for _ in range(3)]
        #     c1, c2 = (int(box[i][0]), int(box[i][1])), (int(box[i][2]), int(box[i][3]))
        #     cv2.rectangle(img, c1, c2, color, tl, lineType=cv2.LINE_AA) # object bounding box
        #     tf = max(tl -1, 1)
        #     t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        #     c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        #     cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled box for labels
        #     cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

        # raw_img = spot_webrtc.cvImage.copy()

        # self.publish_image_to_ros(raw_img)
        # self.publish_yolo_img_to_ros(img)
        # self.publish_bbox(box, cat, conf)
        # self.publish_sem_label(self.model, cat)

    def raw_img_callback(self, event):
        # publish ros image
        # print("publish ros img")
        if self.img_received:
            self.rgb = spot_webrtc.rgbImage.copy()
            self.img = spot_webrtc.cvImage.copy()
            self.publish_image_to_ros(self.img)

    def odom_pub_timer_callback(self, event):
        start_time = rospy.Time.now()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"

        position, quaternion = self.get_location()
        # qx, qy, qz, qw = quaternion.as_quat()
        # Set position
        pose_msg.pose.position.x = position.x
        pose_msg.pose.position.y = position.y
        pose_msg.pose.position.z = position.z

        # Set orientation
        pose_msg.pose.orientation.x = quaternion.x
        pose_msg.pose.orientation.y = quaternion.y
        pose_msg.pose.orientation.z = quaternion.z
        pose_msg.pose.orientation.w = quaternion.w

        # Set odom pos
        odom_msg.pose.pose.position.x = position.x
        odom_msg.pose.pose.position.y = position.y
        odom_msg.pose.pose.position.z = position.z

        # Set odom orientation
        odom_msg.pose.pose.orientation.x = quaternion.x
        odom_msg.pose.pose.orientation.y = quaternion.y
        odom_msg.pose.pose.orientation.z = quaternion.z
        odom_msg.pose.pose.orientation.w = quaternion.w

        # rospy.loginfo(pose_msg)
        self.pose_pub.publish(pose_msg)

        # rospy.loginfo(odom_msg)
        self.odom_pub.publish(odom_msg)
        # print(f"odom pub time: {rospy.Time.now() - start_time}")

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
        #dyaw = self.get_desired_heading(dx, dy)
        
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
            # rospy.loginfo("no command id, skip move status check")
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

    def get_desired_heading(self, dx, dy):
        position, quaternion = self.get_location()
        x = dx - position.x
        y = dy - position.y
        dyaw = math.atan2(y, x)
        return dyaw

    def get_location(self):
        global robot_state_client

        curr_transforms = spot.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
        vision_tform_body = get_vision_tform_body(curr_transforms)

        position = vision_tform_body.position
        rot_quaternion = vision_tform_body.rotation
        return position, rot_quaternion

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

    def publish_image_to_ros(self,cv_img):
        try:
            ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            ros_img.header.stamp = rospy.Time.now()
            self.img_pub.publish(ros_img)
        except CvBridgeError as e:
            print(e)

    def publish_yolo_img_to_ros(self,cv_img):
        try:
            ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            self.yolo_img_pub.publish(ros_img)
        except CvBridgeError as e:
            print(e)

    def publish_bbox(self, box, cat, conf):
        bbox_msg = Detection2DArray()
        bbox_msg.detections = []

        for i in range(len(box)):
            bbox = Detection2D()

            bbox.bbox.center.x = (int(box[i][0]) + int(box[i][2])) / 2
            bbox.bbox.center.y = (int(box[i][1]) + int(box[i][3])) / 2
            bbox.bbox.size_x = abs(int(box[i][2]) - int(box[i][0]))
            bbox.bbox.size_y = abs(int(box[i][3]) - int(box[i][1]))

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(cat[i])
            hypothesis.score = conf[i]
            bbox.results.append(hypothesis)
            
            bbox_msg.detections.append(bbox)
            bbox_msg.header.stamp = rospy.Time.now()
            bbox_msg.header.frame_id = 'yolo_bbox'

        self.bbox_pub.publish(bbox_msg)

    def publish_sem_label(self, model, cat):
        labels_msg = SemanticLabel()
        labels_msg.ids = []
        labels_msg.labels = []

        for i in range(len(cat)):
            cat_id = int(cat[i])
            label = model.names[int(cat[i])]

            labels_msg.ids.append(cat_id)
            labels_msg.labels.append(label)

        self.label_pub.publish(labels_msg)

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
