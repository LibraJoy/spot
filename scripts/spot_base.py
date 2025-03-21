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
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.frame_helpers import get_vision_tform_body
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, get_se2_a_tform_b
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit, blocking_selfright
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from scipy.spatial.transform import Rotation as R
import numpy as np
import torch
import os
import random
import threading
from spot.yolo_sam2 import SAM2
# yolo v8
import PIL.Image
import cv2
import PIL
import torch 
import numpy as np

from ultralytics import YOLO
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
import tf2_ros

detection_model = ["yolov8", "yolov7-sam2"]
class yolo_seg:
    def __init__(self):
        self.w_org = 1280
        self.h_org = 720
        ## new - add the GPU as the device
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        print(f"Yolo device: {self.device}")
        self.model = YOLO("/root/spot_ws/src/spot/models/yolov8m-seg.pt")
        self.model.to(self.device)
        print(f"model is on device: {self.model.device}")
        self.last_time = None
        ## new
        self.img_resized = None
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber('/spot_image', Image, self.image_callback)
        self.yolo_vis_pub = rospy.Publisher('/yolo/visualization', Image, queue_size=10)
        self.yolo_detect_pub = rospy.Publisher('/yolo/detection', Detection2DArray, queue_size=10)
        self.yolo_mask_pub = rospy.Publisher('/yolo/mask', Image, queue_size=10)
        self.img_data = None
        # self.obj_label_of_interest = [56, 57, 60, 62]

    def plot(self, results):
        for i, r in enumerate(results):
            # Plot results image
            im_bgr = r.plot()  # BGR-order numpy array
            im_rgb = PIL.Image.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

            # Show results to screen (in supported environments)
            r.show()

            # Save results to disk
            r.save(filename=f"results{i}.jpg")

    def publish_results(self, results):
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.img_data.header.stamp
        detection_array.header.frame_id = "yolo_result"
        for i, r in enumerate(results): # i always 0
            # publish visualizaiton image
            im_bgr = r.plot()  # BGR-order numpy array
            ros_img = self.bridge.cv2_to_imgmsg(im_bgr, encoding="bgr8")
            self.yolo_vis_pub.publish(ros_img)

            # publish detection results
            
            if r.boxes.cls.shape[0] == 0:
                return
            # print("r.boxes.cls.shape: ", r.boxes.cls.shape)
            # make an empty mask
            mask_base = np.zeros((self.h_org, self.w_org), dtype=np.uint8)
            for j in range(r.boxes.cls.shape[0]): # number of bounding boxes in current frame
                # if int(r.boxes.cls[j]) not in self.obj_label_of_interest:
                #     continue
                detection = Detection2D()
                result = ObjectHypothesisWithPose()
                result.id = int(r.boxes.cls[j])
                result.score = float(r.boxes.conf[j])
                detection.results.append(result)
                detection.bbox.center.x = r.boxes.xywh[j][0]
                detection.bbox.center.y = r.boxes.xywh[j][1]
                detection.bbox.size_x = r.boxes.xywh[j][2]
                detection.bbox.size_y = r.boxes.xywh[j][3]
                mask = r.masks.data[j,:,:]
                mask = mask.cpu().numpy().astype(np.uint8)
                # any pixel is not 0 in either mask_base or mask, set it to 1. else set it to 0
                mask = cv2.resize(mask, (self.w_org, self.h_org))
                # if result.id == 56:
                mask_base = np.where(mask_base + mask > 0, 1, 0)
                mask = mask * 255
                # import pdb; pdb.set_trace()
                mask = PIL.Image.fromarray(mask)
                mask = self.bridge.cv2_to_imgmsg(np.array(mask), encoding="mono8")
                detection.source_img = mask
                detection_array.detections.append(detection)
            
            mask_base = (mask_base * 255).astype(np.uint8)
            mask_base = PIL.Image.fromarray(mask_base)
            mask_base = self.bridge.cv2_to_imgmsg(np.array(mask_base), encoding="mono8")
            self.yolo_mask_pub.publish(mask_base)

            
            self.yolo_detect_pub.publish(detection_array)
        
 

    def image_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_data = data
        except CvBridgeError as e:
            print(e)

        results = self.model.predict(source=cv_img, save=False, save_txt=False, stream=True, verbose=False)
        # self.plot(results)
        self.publish_results(results)

    # make bounding box and semantic mask msg from yolo results->Detection2DArray
    def make_msg(self, results):
        print("make_msg")
        detection_array = Detection2DArray()
        for i, r in enumerate(results):
            print("get r")
            detection = Detection2D()
            detection.header.stamp = self.img_data.header.stamp
            detection.header.frame_id = "yolo_result"
            for j in range(r.boxes.cls.shape[0]): # number of bounding boxes in current frame
                box = BoundingBox2D()
                box.results[0].id = r.boxes.cls[j]
                box.results[0].score = r.boxes.conf[j]
                box.center.x = r.boxes.xywh[j][0]
                box.center.y = r.boxes.xywh[j][1]
                box.size_x = r.boxes.xywh[j][2]
                box.size_y = r.boxes.xywh[j][3]
                detection.results.append(box)
            detection_array.detections.append(detection)
        return detection_array



robot = None
robot_state_client = None
robot_command_client = None

# For check repeated path_msg
hist_path = None

p = None
webrtc_thread = None
shutdown_flag = None
reached_goal = None
goal_mode = ["subscribe", "click"]

class spotMoveBase:
    def __init__(self):
        
        # params:
        self.reach_tolerance = 0.15

        self.cmd_id = None
        self.rotate_cmd_id = None
        self.rotate_flag = False # rotation needed = True
        self.img = None
        self.img_received = False
        self.heading = None
        self.v_lin = 0.5
        self.v_ang = 0.4
        self.mobility_params = self.set_mobility_params(self.v_lin, self.v_lin, self.v_ang, -self.v_lin, -self.v_lin, -self.v_ang)
        self.position = None

        # goal mode flag
        self.goal_mode = "subscribe"

        # detection model
        # self.detection_model = "yolov8"
        self.detection_model = "yolov7-sam2"

        # ROS
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.goal = [0., 0., 0.] # x,y,yaw
        # rospy.Subscriber("/navigation_SPOT/waypoints", PoseStamped, self.goal_pose_sub_callback)
        rospy.Subscriber("/spot/waypoint", PoseStamped, self.goal_pose_sub_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_click_callback)
        self.pose_pub = rospy.Publisher('/spot/pose', PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/spot/odom', Odometry, queue_size=10)
        self.img_pub = rospy.Publisher('/spot_image', Image, queue_size=10)

        rospy.Timer(rospy.Duration(0.03), self.odom_pub_timer_callback)
        rospy.Timer(rospy.Duration(0.05), self.move_status_check_timer_callback)

        # self.yolo = yolo_seg()
        if self.detection_model == "yolov8":
            self.yolo = yolo_seg()
        elif self.detection_model == "yolov7-sam2":
            # SAM2 instance
            self.sam2 = SAM2()

        self.bridge = CvBridge()
        spot.set_screen('pano_full')
        self.startMonitor(spot.hostname, spot.robot)

        rospy.Timer(rospy.Duration(0.1), self.raw_img_callback)

    def set_mobility_params(self, max_x_vel, max_y_vel, max_yaw_vel, min_x_vel, min_y_vel, min_yaw_vel):
        speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=max_x_vel, y=max_y_vel), angular=max_yaw_vel), min_vel=SE2Velocity(linear=Vec2(x=min_x_vel, y=min_y_vel), angular=min_yaw_vel))
        mobility_params = spot_command_pb2.MobilityParams(vel_limit=speed_limit)
        return mobility_params

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

        while not rospy.is_shutdown():
            c = cv2.waitKey(1)
            if c == 27:
                break
            if not webrtc_thread.is_alive():
                break
            elif spot_webrtc.frameCount == 0:
                tm1 = time.time()
                # print("-------------------------- frame count = 0 ----------------------")
                if spot_webrtc.frameR is None:
                    # print("-------------------- NO FRAME RECEIVED FROM QUEUE --------------------")
                    pass
            else:
                print("-----------------------IMAGE QUEUE READY-----------------------")
                self.img_received = True
                break

    def raw_img_callback(self, event):
        # publish ros image
        # print("publish ros img")
        if self.img_received:
            # self.rgb = spot_webrtc.rgbImage.copy()
            self.img = spot_webrtc.cvImage.copy()
            self.publish_image_to_ros(self.img)
            self.img_yolo = spot_webrtc.rgbImage.copy()
            # convert img_yolo to torch and move to cuda
            # self.img_yolo = cv2.cvtColor(self.img_yolo, cv2.COLOR_RGB2BGR)
            # self.img_yolo = torch.from_numpy(self.img_yolo).unsqueeze(0).float().to('cuda')

    def odom_pub_timer_callback(self, event):
        start_time = rospy.Time.now()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "spot_base"

        position, quaternion = self.get_location()
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
        self.position = [position.x, position.y, position.z]
        
        # broadcast transform from map to spot_base
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = "map"
        tf.child_frame_id = "spot_base"
        tf.transform.translation.x = position.x
        tf.transform.translation.y = position.y
        tf.transform.translation.z = position.z
        tf.transform.rotation.x = quaternion.x
        tf.transform.rotation.y = quaternion.y
        tf.transform.rotation.z = quaternion.z
        tf.transform.rotation.w = quaternion.w
        self.tf_broadcaster.sendTransform(tf)

    def goal_pose_sub_callback(self, msg):
        self.goal_mode = "subscribe"
        global robot_command_client 
        global robot_state_client
        frame_name = VISION_FRAME_NAME
        
        # print("in goal pose sub callback")
        if self.goal[0] == msg.pose.position.x and self.goal[1] == msg.pose.position.y:
            rospy.loginfo("subscribed goal has not changed, do not upddate command")
            return
        #  Check distance between current position and goal
        self.goal = [msg.pose.position.x, msg.pose.position.y, 0]
        [dx, dy, dyaw] = self.goal
        current_heading, current_x, current_y, self.heading = self.get_desired_heading(dx, dy)
        
        if math.sqrt((current_x - self.goal[0])**2 + (current_y - self.goal[1])**2) < self.reach_tolerance:
            rospy.loginfo("goal is too close to current position, skip move command")
            return
        
        rospy.loginfo(f"waypoint: x: {self.goal[0]}, y: {self.goal[1]}")

        dyaw = self.heading
        if abs(current_heading - self.heading) < 0.2:
            self.send_move_command()
        else:
            # rotation required when a new goal is received
            self.rotate_flag = True
            rotate_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=current_x, goal_y=current_y, goal_heading=dyaw,
                frame_name=frame_name, params=self.mobility_params)

            # calculate rotate end time and send rotate cmd
            rotate_end_time = 1.5*(abs(current_heading - self.heading)/self.v_ang)
            rotate_end_time = min(10.0, rotate_end_time)
            # print("rotate end time: ", rotate_end_time)
            self.rotate_cmd_id = spot.robot_command_client.robot_command(lease=None, command=rotate_cmd,
                                                        end_time_secs=time.time() + rotate_end_time)
            print(f"rotation command request sent: {dyaw}")

    def goal_pose_click_callback(self, msg):
        self.goal_mode = "click"
        global robot_command_client 
        global robot_state_client
        frame_name = VISION_FRAME_NAME
        
        # print("in goal pose sub callback")
        if self.goal[0] == msg.pose.position.x and self.goal[1] == msg.pose.position.y:
            rospy.loginfo("subscribed goal has not changed, do not upddate command")
            return
        #  Check distance between current position and goal
        self.goal = [msg.pose.position.x, msg.pose.position.y, 0]
        [dx, dy, dyaw] = self.goal
        current_heading, current_x, current_y, self.heading = self.get_desired_heading(dx, dy)
        
        if math.sqrt((current_x - self.goal[0])**2 + (current_y - self.goal[1])**2) < self.reach_tolerance:
            rospy.loginfo("goal is too close to current position, skip move command")
            return
        
        rospy.loginfo(f"waypoint: x: {self.goal[0]}, y: {self.goal[1]}")

        # send rotation command
        dyaw = self.heading
        if abs(current_heading - self.heading) < 0.2:
            self.send_move_command()
        else:
            self.rotate_flag = True
            rotate_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=current_x, goal_y=current_y, goal_heading=dyaw,
                frame_name=frame_name, params=self.mobility_params)

            # calculate rotate end time and send rotate cmd
            rotate_end_time = 1.5*(abs(current_heading - self.heading)/self.v_ang)
            rotate_end_time = min(10.0, rotate_end_time)
            self.rotate_cmd_id = spot.robot_command_client.robot_command(lease=None, command=rotate_cmd,
                                                        end_time_secs=time.time() + rotate_end_time)
            print("rotate end time: ", rotate_end_time)
            print(f"rotation command request sent: {dyaw}")

    def send_move_command(self):
        global robot_command_client 
        global robot_state_client
        frame_name = VISION_FRAME_NAME
        [dx, dy, dyaw] = self.goal
        dyaw = self.heading

        # send move command after rotated to desired heading
        if self.rotate_flag == False:
            robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=dx, goal_y=dy, goal_heading=dyaw,
                frame_name=frame_name, params=self.mobility_params)
            # calculate move end time and send move cmd
            end_time = 2.0 * (math.sqrt((dx - self.position[0])**2 + (dy - self.position[1])**2)/self.v_lin)
            end_time = min(20.0, end_time)
            end_time = max(4.0, end_time)
            print(f"move end time: {end_time}")
            self.cmd_id = spot.robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                        end_time_secs=time.time() + end_time)
            print(f"movement command request sent: {self.goal}") 

    def move_status_check_timer_callback(self, event):
        if not self.rotate_cmd_id and not self.cmd_id:
            # rospy.loginfo("no rotation and movement commands, skip move status check")
            return
        # elif self.rotate_cmd_id:
                # print(f"check rotation command id {self.rotate_cmd_id}")
        # elif self.cmd_id:
                # print(f"check move command id {self.cmd_id}")

        # check rotation status
        if self.rotate_cmd_id:
            # print(f"current desired heading: {self.heading}")
            current_heading = self.get_desired_heading(self.goal[0], self.goal[1])[0]
            # print(f"current heading: {current_heading}")

            rot_feedback = spot.robot_command_client.robot_command_feedback(self.rotate_cmd_id)
            rot_mobility_feedback = rot_feedback.feedback.synchronized_feedback.mobility_command_feedback
            rot_traj_feedback = rot_mobility_feedback.se2_trajectory_feedback

            if rot_mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                # print(f"current rotation command id {self.rotate_cmd_id}, status: {rot_mobility_feedback.status}")
                # if rotation command time out, print failed message and send move command
                if rot_mobility_feedback.status == 3:
                    # calculate the difference between current heading and desired heading
                    diff = abs(current_heading - self.heading)
                    if diff > 0.1:
                        # if rotation command time out and not reaching the goal, print failed message
                        print(f"rotate cmd {self.rotate_cmd_id} failed due to time out. Send move cmd.")
                    else:
                        rospy.logwarn("Facing the desired heading within tolerance now. Send move cmd.")
                    # clear rotation command id
                    self.rotate_flag = False
                    self.rotate_cmd_id = None
                    # send move command
                    self.send_move_command()
            
            # send move command after rotation is done
            if (rot_traj_feedback.status == rot_traj_feedback.STATUS_AT_GOAL and
                    rot_traj_feedback.body_movement_status == rot_traj_feedback.BODY_STATUS_SETTLED):
                rospy.logwarn("Facing the desired heading now.")

                # clear rotation command id
                self.rotate_flag = False
                self.rotate_cmd_id = None
                # send move command
                self.send_move_command()

        # check movement status
        if not self.cmd_id:
            # rospy.loginfo("no command id, skip move status check")
            return
        if self.cmd_id:
            feedback = spot.robot_command_client.robot_command_feedback(self.cmd_id)
            mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
            traj_feedback = mobility_feedback.se2_trajectory_feedback
            if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                # print(f"current move command id {self.cmd_id}, status: {mobility_feedback.status}")

                if mobility_feedback.status == 3:
                    # set reached goal tolerance to 0.15m
                    diff = math.sqrt((self.position[0] - self.goal[0])**2 + (self.position[1] - self.goal[1])**2)
                    if diff > self.reach_tolerance:
                        # if move command time out and not reaching the goal, print failed message
                        print(f"move cmd {self.cmd_id} failed due to time out. distance to goal is {diff}")
                    else:
                        rospy.logwarn("Reached goal within tolerance now.")
                    # clear move command id otherwise it's keep printing msg
                    self.cmd_id = None
            
            if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                    traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
                rospy.logwarn("Arrived at the goal.")
                self.cmd_id = None
                # in click mode, if goal is origin, sit
                if self.goal_mode == "click":
                    if (math.sqrt(self.goal[0]**2 + self.goal[1]**2) < 0.3):
                        spot.sit()
                        rospy.logwarn("click mode goal is origin, sit")

    def get_desired_heading(self, dx, dy):
        position, quaternion = self.get_location()
        current_x = position.x
        current_y = position.y
        current_rot = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        current_yaw = current_rot.as_euler('xyz', degrees=False)[2]

        x = dx - current_x
        y = dy - current_y
        goal_yaw = math.atan2(y, x)
        # print(f"current yaw: {current_yaw}, goal yaw: {goal_yaw}")

        diff = abs(goal_yaw - current_yaw)
        if diff > math.pi:
            diff = 2*math.pi - diff
        # print(f"diff: {diff}")
        if math.pi/8 < diff < 3*math.pi/4:
            dyaw = goal_yaw
        else:
            dyaw = current_yaw
        # print(f"desired heading: {dyaw}")
        return current_yaw, current_x, current_y, dyaw

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
            # print("test time delay")
            # print(rospy.Time.now())
            ros_img.header.stamp = rospy.Time.now() - rospy.Duration(0.25)
            # print(ros_img.header.stamp)
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
