# Spot Keyboard Teleop (`spot_teleop_keyboard`)

This ROS 2 node provides an advanced WASD keyboard controller for the Boston Dynamics Spot robot. It allows human operators to manually navigate the robot, adjust its body pose (height, pitch, roll), trigger basic actions (stand, sit), and initiate emergency stops (E-Stop).

It's designed to be used in conjunction with the `spot_base` node.

## Quick Start Guide

### 1. Dependencies
The keyboard listener requires the `pynput` library.
```bash
pip install pynput
```

### 2. Building
From your ROS 2 workspace root:
```bash
cd ~/ros2_ws
colcon build --packages-select spot_ros2
source install/setup.bash
```

### 3. Execution
The teleoperation system typically requires three separate terminals to run safely:

#### Terminal 1: E-Stop (Required for API authorization)
*You must keep this running as the primary emergency kill switch.*
```bash
cd ~/ros2_ws/src/spot_ros2/spot_ros2
python3 estop_nogui.py
```

#### Terminal 2: Spot Base Driver
*Handles all Spot physical API communication.*
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run spot_ros2 spot_base
```

#### Terminal 3: Keyboard Teleop Interface
*This terminal will display the control UI and intercept your keystrokes.*
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run spot_ros2 spot_teleop_keyboard
```

## Control Mapping

Once running, Terminal 3 becomes the control interface. Use the following keys:

```text
Movement:
  W/S = Forward/Backward
  A/D = Strafe Left/Right
  Q/E = Rotate Left/Right 

Body Pose:
  Shift/Ctrl = Raise/Lower Height
  Arrow Keys = Adjust Pitch and Roll
  Home       = Reset Body Pose to Default
  </>        = Yaw with legs fixed 

Actions:
  Z = Command Spot to Sit
  X = Command Spot to Stand
  Space = E-STOP (Software Emergency Stop)
  ESC = Exit Teleop Node

Speed:
  + = Increase Movement Velocity Multiplier 
  - = Decrease Movement Velocity Multiplier

Modes:
  T = Toggle Control Mode (Switches between Manual Keyboard Control and Autonomous /cmd_vel pass-through)
```

## Troubleshooting

- **"pynput not installed" Error:** Run `pip install pynput`.
- **Spot doesn't move when I press W/A/S/D:**
  1. Make sure your Terminal 3 window is active (in focus) to capture keystrokes.
  2. Check Terminal 1 to ensure the E-Stop script hasn't tripped.
  3. Verify the `spot_base` node is successfully publishing to `/odom`.
  4. Press `T` to ensure you are in "Keyboard Control" mode, not "Autonomy".

**For deeper technical details, see:**
- [TELEOP_KEYBOARD.md](TELEOP_KEYBOARD.md) (Architecture Details)
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) (Implementation Summary)
