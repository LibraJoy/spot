#!/bin/bash
# Function to handle SIGINT (Ctrl+C) signal

cleanup() {
    echo "SIGINT received, terminating processes..."
    kill $KEYBOARD_CONTROL_PID
    kill $ISAAC_SIM_PID
    exit 0
}
# Trap SIGINT and call the cleanup function
trap 'cleanup' SIGINT

# Source environment scripts
source ~/.bashrc
source ~/spot_ws/devel/setup.bash

# roslaunch ouster_ros driver.launch sensor_hostname:=os-122412000058.local lidar_mode:=1024x20 timestamp_mode:=TIME_FROM_ROS_TIME
roslaunch ouster_ros driver.launch sensor_hostname:=os-122412000058.local lidar_mode:=1024x10 timestamp_mode:=TIME_FROM_ROS_TIME

# # Open a new terminal and run the register node
# gnome-terminal -- bash -c "source ~/.bashrc; source ~/spot_ws/devel/setup.bash; rosrun lidar_cam_calibrater register_node; exec bash" &
