# Spot Base Node (`spot_base.py`)

The `spot_base` node acts as the central communication bridge between your ROS 2 network and the Boston Dynamics Spot robot API. 

## Node Responsibilities
1.  **Movement Translation**: Subscribes to velocity (`/spot/cmd_vel`) and navigation goal (`/spot/waypoint`, `/goal_pose`) commands, translating them into API-level commands (e.g., `RobotCommandBuilder`).
2.  **State Publishing**: Constantly queries the robot's physical state (Position, Odometry, Battery) and publishes them as standard ROS 2 messages.
3.  **Vision Pipeline**: Connects to the Spot CAM payload (via WebRTC/ImageClient, depending on configuration) and publishes the camera feed (`/spot_image`). Note: It also contains publisher endpoints for a YOLO perception pipeline (e.g., bounding boxes, masks, and visualizations) if integrated.
4.  **Teleoperation States**: Listens to dedicated topics to manage the robot's standing/sitting state, body posture, and whether a teleop override is currently active.

---

##  Running the Node

**On a local dev machine:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run spot_ros2 spot_base
```

**On Spot's Onboard PC (cerlab72):**
To quickly start the node on the robot's companion computer, use the provided aliases:
```bash
source ~/run_spot_ros2.sh
```
*If you need to configure the IP routing table to allow LiDAR network traffic to route through Spot, run this first:*
```bash
source ~/add_LiDAR_spot_ip.sh
```

---

##  Subscribed Topics (Inputs)

The `spot_base` node listens to the following topics to control the robot's movement and state:

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/spot/cmd_vel` | `geometry_msgs/msg/Twist` | Linear and angular velocity commands (typically from `spot_teleop_keyboard` or a nav stack). |
| `/spot/body_pose` | `geometry_msgs/msg/Twist` | Adjusts the robot's 3D footprint posture (Roll, Pitch, Yaw, and Height offset). |
| `/spot/waypoint` | `geometry_msgs/msg/PoseStamped` | Receives a target waypoint for autonomous navigation (API-driven). |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` |  used via Rviz "2D Goal Point" clicks for simple point-to-point navigation. |
| `/spot/sit` | `std_msgs/msg/Bool` | Commands the robot to execute the sit sequence. |
| `/spot/stand` | `std_msgs/msg/Bool` | Commands the robot to execute the stand sequence. |
| `/spot/teleop_active` | `std_msgs/msg/Bool` | Tracks if manual teleop is currently overriding autonomous commands (/goal_pose). |
| `/spot_image` | `sensor_msgs/msg/Image` | (Internal loopback) Subscribes to its own image feed for YOLO processing if enabled. |

---

##  Published Topics (Outputs)

The node queries the robot API at a set frequency and publishes the following telemetry and perception data:

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/spot/odom` | `nav_msgs/msg/Odometry` | The estimated odometry (position and velocity) in the `odom` frame. |
| `/spot/pose` | `geometry_msgs/msg/PoseStamped` | The 3D pose of the robot's body. |
| `/spot/battery` | `sensor_msgs/msg/BatteryState` | Battery percentage and power status. |
| `/spot/teleop_feedback` | `std_msgs/msg/String` | Feedback messages regarding teleoperation state changes. |
| `/spot_image` | `sensor_msgs/msg/Image` | The raw panoramic/camera stream retrieved from the Spot CAM payload (WebRTC). |
| `/yolo/visualization` | `sensor_msgs/msg/Image` | (Optional) Image stream with YOLO bounding boxes drawn over it. |
| `/yolo/detection` | `vision_msgs/msg/Detection2DArray` | (Optional) Array of 2D bounding boxes and class IDs from YOLO. |
| `/yolo/mask` | `sensor_msgs/msg/Image` | (Optional) Semantic segmentation masks if using SAM/YOLO-Seg. |
