# Spot ROS 2 Integration (`spot_ros2`)

This ROS 2 package provides the essential nodes to connect, control, and receive data from a Boston Dynamics Spot robot via its API.

## 🔧 Hardware & Network Connection

Before launching any ROS 2 nodes, ensure your machine is properly connected to Spot's network and authorized via its API.

**Detailed Connection Guide:**  
🔗 [Connecting to SPOT (Notion Guide)](https://www.notion.so/Connecting-to-SPOT-31f68f16f8a68029bde2fa6a647e64a6?source=copy_linkhttps://www.notion.so/Connecting-to-SPOT-31f68f16f8a68029bde2fa6a647e64a6?source=copy_linkhttps://www.notion.so/Connecting-to-SPOT-31f68f16f8a68029bde2fa6a647e64a6?source=copy_link)

---

##  Core Nodes

### 1. Spot Base Node (`spot_base`)
The `spot_base` node is the core driver that handles all bidirectional communication with the Spot API. It manages the robot's physical state, processes movement commands, and publishes sensory data (including Spot CAM images and odometry).

*   **Quick Run (Local)**: 
    ```bash
    ros2 run spot_ros2 spot_base
    ```
*   **Quick Run (on Spot's Onboard PC - cerlab72)**:
    ```bash
    # To add necessary IP routes for (LiDAR and spot itself):
    source ~/add_LiDAR_spot_ip.sh

    source ~/run_spot_ros2.sh
    # enter credientials. activate estop prior to spot_base!
    ```
*   **Documentation**: See the full topic list and detailed configuration in [docs/spot_base.md](docs/spot_base.md).

### 2. Spot Teleop Keyboard (`spot_teleop_keyboard`)
An WASD keyboard controller for Spot. It features an integrated E-Stop, height/pitch/roll body posing, and a seamless toggle between manual keyboard control and autonomous `/cmd_vel` pass-through.

To control spot remotely, use this node through ssh. 

**CAUTION**: This node will listen to keyboard inputs globally. Shut it down using `ESC` before typing anything else!

*   **Quick Run**: 
    ```bash
    # run spot_base first!
    ros2 run spot_ros2 spot_teleop_keyboard
    ```
*    See installation and **controls** in [docs/spot_teleop_keyboard.md](docs/spot_teleop_keyboard.md).


**Documentation**:

-    [Teleop Keyboard Details](docs/TELEOP_KEYBOARD.md)
-    [Implementation Summary](docs/IMPLEMENTATION_SUMMARY.md)

---

## 🌐 Multi-Machine ROS 2 Setup

To communicate with Spot across multiple physical machines (e.g., your laptop and Spot's onboard PC), you must ensure ROS 2 networking is configured correctly.

1.  **Network Subnet**: All machines must be on the same network (e.g., local Wi-Fi, Ethernet switch, or a VPN like Tailscale).
2.  **Domain ID**: Leave `ROS_DOMAIN_ID` blank or export the exact same `ROS_DOMAIN_ID` on every machine (e.g., `export ROS_DOMAIN_ID=10`).
3.  **RMW Implementation**: Ensure all machines use the same DDS implementation. You can check your current implementation using `ros2 doctor --report` (look for `rmw_fastrtps_cpp` or `rmw_cyclonedds_cpp`).

**Network Debugging Tools:**
If nodes are not discovering each other, test UDP multicast connectivity between machines:
*   *On Machine A*: `ros2 multicast receive`
*   *On Machine B*: `ros2 multicast send`

If it says "received 1 package", your connection is good.

An useful reset script is at `cerlab@cerlab72:~/Desktop/ros2_dds_restore_default.sh`

However, this script will **return all settings to default**, so use with caution.

---

## 🐳 Docker Configuration

When running ROS 2 nodes inside a Docker container, the container must be able to discover nodes on the host network.

### 1. Direct `docker run` (with X11 Forwarding)
If you need GUI applications (like Rviz or OpenCV windows) while running a standalone container, use the following command to enable host networking and X11 forwarding:
```bash
xhost +local:root
docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="ROS_DOMAIN_ID=10" \
    osrf/ros:humble-desktop bash

    # osrf/ros:humble-desktop is the official humble image, you can replace it with your own image name
```

If you are also using ssh:
```bash
ssh -X <user>@xxx.xxx.xxx.xxx
```
-X will automatically forward GUIs to the remote computer. If -X doesn't work, use -Y instead.

### 2. Docker Compose
In your `docker-compose.yml`, you **must** use `network_mode: "host"` and pass the environment variables:
```yaml
services:
  ros2_spot:
    image: osrf/ros:humble-desktop
    network_mode: "host"
    environment:
      - ROS_DOMAIN_ID=10
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
