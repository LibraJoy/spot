# Spot ROS1 Package

ROS Noetic driver and control stack for the Boston Dynamics Spot robot.

---

## Table of Contents
- [Environment Setup](#environment-setup)
- [Connect to Spot](#connect-to-spot)
- [Control Modes and Priority](#control-modes-and-priority)
- [Running Nodes](#running-nodes)
- [Keyboard Teleop Controls](#keyboard-teleop-controls)
- [Published / Subscribed Topics](#published--subscribed-topics)

---

## Environment Setup

Add to `~/.bashrc`:
```bash
export BOSDYN_CLIENT_USERNAME=user
export BOSDYN_CLIENT_PASSWORD=scgau6g5w987
export SPOT_HOSTNAME="192.168.80.3"   # adjust per connection method below
```

Install dependencies:
```bash
mkdir -p spot_ws/src
cd spot_ws/src
git clone https://github.com/LibraJoy/spot.git
cd spot/src/spot
pip install -r requirements.txt
pip install pynput          # required for keyboard teleop
cd ~/spot_ws
catkin_make
```

---

## Connect to Spot

| Method | IP Address |
|--------|-----------|
| Ethernet via Spot Dock | `192.168.50.3` |
| Ethernet via rear connector | `10.0.0.3` |
| Spot Wi-Fi | `192.168.80.3` |

To switch connection method:
```bash
# In ~/.bashrc
export SPOT_HOSTNAME="<IP ADDRESS>"

# In spot_ws/src/spot/src/spot/estop_nogui.py
argv.append("<IP ADDRESS>")
```

**Always start the E-Stop before any other node:**
```bash
cd ~/spot_ws/src/spot/src/spot
python estop_nogui.py
```

---

## Control Modes and Priority

Three control modes are supported. **Priority is fixed** — higher modes override lower ones while active:

| Priority | Mode | Trigger | Description |
|----------|------|---------|-------------|
| **1 (Highest)** | Keyboard Teleop | `spot_teleop_keyboard.py` running | Direct velocity commands; overrides all other inputs while the node is alive |
| **2** | RViz Click | `2D Nav Goal` in RViz → `/move_base_simple/goal` | Single-goal trajectory commands |
| **3 (Lowest)** | Autonomous Planner | `autonomous_robot` package → `/spot/waypoint` | Exploration and navigation goals |

**How the override works:** `spot_teleop_keyboard.py` publishes a 1 Hz heartbeat on `/spot/teleop_active`. While the heartbeat is fresh (< 2 s old), `spot_base.py` silently drops all incoming `/spot/waypoint` and `/move_base_simple/goal` messages. When the teleop node exits, autonomous control resumes automatically after ~2 s.

**Use teleop for:**
- Emergency stops during autonomous runs
- Manual positioning for LiDAR-camera calibration or SlideSLAM initialization
- Operations where autonomous planning is not desired

---

## Running Nodes

### Core

```bash
# Terminal 1 — E-Stop (always first)
cd ~/spot_ws/src/spot/src/spot && python estop_nogui.py

# Terminal 2 — Main driver (odometry, waypoint control, YOLO detection)
rosrun spot spot_base.py

# Terminal 3 — Camera image publisher
rosrun spot img_pub.py

# Terminal 4 — Pose and odometry republisher
rosrun spot publisher.py
```

### Keyboard Teleop (highest priority)

```bash
# In a new terminal — teleop immediately overrides planner and RViz
rosrun spot spot_teleop_keyboard.py
```

Close the teleop terminal (or press ESC) to hand control back to the planner / RViz.

---

## Keyboard Teleop Controls

The teleop node uses `termios`/stdin so it works over **SSH without X11**.

| Key | Action |
|-----|--------|
| `W` / `S` | Forward / Backward |
| `A` / `D` | Strafe Left / Right |
| `Q` / `E` | Rotate Left / Right |
| `R` / `F` | Increase / Decrease body height (limit ±10 cm) |
| `↑` / `↓` | Pitch forward / backward |
| `←` / `→` | Roll left / right |
| `,` / `.` | Body yaw left / right (limit ±25°) |
| `H` | Reset body pose to default |
| `Z` | Sit |
| `X` | Stand |
| `Space` | Emergency stop (zero all velocity) |
| `+` / `-` | Increase / Decrease speed |
| `ESC` or `Ctrl+C` | Exit teleop (returns control to planner/RViz) |

**Multi-key movement:** W+Q, A+E, etc. are combinable — each movement key tracks its own hold timestamp (250 ms expiry). Terminal key-repeat keeps it active while held; releasing a key causes it to expire within ~250 ms.

Body pose keys are incremental per press and are not affected by the hold timeout.

Default speed: **0.3 m/s linear**, **0.2 rad/s angular**. Range: 0.1 – 1.5 m/s.

---

## Published / Subscribed Topics

### spot_base.py

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Sub | `/spot/waypoint` | `PoseStamped` | Goal from autonomous planner (priority 3) |
| Sub | `/move_base_simple/goal` | `PoseStamped` | Goal from RViz click (priority 2) |
| Sub | `/spot/cmd_vel` | `Twist` | Velocity from teleop (priority 1) |
| Sub | `/spot/teleop_active` | `Bool` | Teleop heartbeat — suppresses lower-priority inputs |
| Sub | `/spot/sit` | `Bool` | Sit command from teleop |
| Sub | `/spot/stand` | `Bool` | Stand command from teleop |
| Sub | `/spot/body_pose` | `Twist` | Body pose from teleop (`linear.z`=height, `angular.xyz`=roll/pitch/yaw) |
| Pub | `/spot/odom` | `Odometry` | Robot odometry at 33 Hz |
| Pub | `/spot/pose` | `PoseStamped` | Robot pose at 33 Hz |
| Pub | `/spot_image` | `Image` | Camera feed at 10 Hz |
| Pub | `/spot/teleop_feedback` | `String` | Feedback messages to teleop node |

### spot_teleop_keyboard.py

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/spot/cmd_vel` | `Twist` | Velocity commands at 10 Hz |
| Pub | `/spot/body_pose` | `Twist` | Body pose on key press |
| Pub | `/spot/teleop_active` | `Bool` | Heartbeat at 1 Hz |
| Pub | `/spot/sit` | `Bool` | Sit on `Z` |
| Pub | `/spot/stand` | `Bool` | Stand on `X` |
| Sub | `/spot/odom` | `Odometry` | Position display |
| Sub | `/spot/teleop_feedback` | `String` | Feedback display |
