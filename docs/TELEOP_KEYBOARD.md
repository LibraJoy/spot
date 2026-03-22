# Spot Keyboard Teleoperation

This package provides keyboard teleoperation control for the Boston Dynamics Spot robot in ROS2.

## Features

- **6-DOF Control**: Full control over X, Y, Yaw, Height, Pitch, and Roll
- **Real-time Display**: Live terminal UI showing status, position, orientation, and battery
- **Safety Features**:
  - Velocity limits
  - Body pose limits
  - Deadman switch (auto-stop)
  - Emergency stop
- **Mode Switching**: Seamlessly switch between autonomous navigation and teleop control

## Installation

### Prerequisites

Install the required Python package:

```bash
pip install pynput
```

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select spot_ros2
source install/setup.bash
```

## Usage

### 1. Start the E-Stop (Required!)

Always run the e-stop first:

```bash
cd ~/ros2_ws/src/spot_ros2/spot_ros2
python3 estop_nogui.py
```

### 2. Start the Base Node

In a new terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run spot_ros2 spot_base
```

This will:
- Connect to Spot
- Publish odometry, images, and battery status
- Handle both autonomous navigation and teleop commands

### 3. Start the Keyboard Teleop Node

In a new terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run spot_ros2 spot_teleop_keyboard
```

You should see a terminal UI with real-time status information.

## Keyboard Controls

### Movement
- **W** - Move forward
- **S** - Move backward
- **A** - Strafe left
- **D** - Strafe right
- **Q** - Rotate left (CCW)
- **E** - Rotate right (CW)

### Body Control
- **Shift** - Increase height (up to +15cm)
- **Ctrl** - Decrease height (down to -15cm)
- **↑** (Up Arrow) - Pitch forward
- **↓** (Down Arrow) - Pitch backward
- **←** (Left Arrow) - Roll left
- **→** (Right Arrow) - Roll right
- **Home** - Reset body pose to default

### Actions
- **Z** - Sit down
- **X** - Stand up
- **Space** - Emergency stop (zeros all velocities)
- **+** or **=** - Increase speed
- **-** or **_** - Decrease speed
- **ESC** - Exit teleop mode

## Terminal Display

The terminal UI shows:

```
====================================================================
                   SPOT KEYBOARD TELEOP
====================================================================

📊 STATUS:
  🔋 Battery:     85.3%
  📍 Position:    X: 1.23m  Y:-0.45m  Z: 0.12m
  🧭 Orientation: R:  2.1°  P: -1.3°  Y: 45.2°

🎮 CONTROL:
  ➡️  Velocity:    Fwd: 0.30 m/s  Left: 0.00 m/s  Rot: 0.00 rad/s
  ⚡ Speed:       0.30 m/s (±: adjust)

🤸 BODY POSE:
  📏 Height:      5.0 cm  (Shift/Ctrl)
  🔄 Roll:        0.0°    (←/→ arrows)
  ↕️  Pitch:       0.0°    (↑/↓ arrows)

⌨️  CONTROLS:
  Movement:  W/S:Fwd/Back  A/D:Left/Right  Q/E:Rotate
  Body:      Shift/Ctrl:Height  Arrows:Pitch/Roll  Home:Reset
  Actions:   Z:Sit  X:Stand  Space:E-STOP  +/-:Speed  ESC:Exit
====================================================================
```

## Safety Limits

The system enforces the following safety limits:

| Parameter | Limit | Notes |
|-----------|-------|-------|
| Linear Velocity | ±1.0 m/s | Adjustable with +/- keys |
| Angular Velocity | ±1.5 rad/s | Adjustable with +/- keys |
| Height Offset | ±15 cm | From default standing height |
| Pitch/Roll | ±23° (0.4 rad) | Safe range for body stability |

## Mode Switching

The system operates in two modes:

### AUTONOMOUS Mode (Default)
- Responds to `/spot/waypoint` and `/goal_pose` navigation commands
- Ignores teleop velocity commands
- Used for autonomous navigation

### TELEOP Mode
- Activated when keyboard teleop node is running
- Responds to velocity and body pose commands
- Automatically returns to AUTONOMOUS mode 1 second after teleop node stops

The mode switch is **automatic** and **seamless**. When you start the teleop keyboard node, it sends heartbeat signals that switch the base node to TELEOP mode. When you exit the teleop node (ESC), the base node returns to AUTONOMOUS mode after a 1-second timeout.

## ROS2 Topics

### Published by Teleop Node
- `/spot/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/spot/body_pose` (geometry_msgs/Twist) - Body pose commands
- `/spot/teleop_active` (std_msgs/Bool) - Heartbeat signal
- `/spot/sit` (std_msgs/Bool) - Sit command trigger
- `/spot/stand` (std_msgs/Bool) - Stand command trigger

### Subscribed by Teleop Node
- `/spot/odom` (nav_msgs/Odometry) - Robot odometry for display
- `/spot/battery` (sensor_msgs/BatteryState) - Battery status
- `/spot/teleop_feedback` (std_msgs/String) - Warnings and feedback

## Troubleshooting

### "pynput not installed" error
```bash
pip install pynput
```

### Spot doesn't respond to commands
1. Check that e-stop is running and not engaged
2. Verify spot_base node is running
3. Check that Spot is powered on and connected
4. Verify network connectivity to Spot

### Terminal UI not displaying correctly
- Ensure your terminal supports ANSI color codes
- Try resizing the terminal window
- Use a modern terminal emulator (e.g., GNOME Terminal, Konsole, iTerm2)

### Mode stuck in TELEOP
- Exit the teleop keyboard node (ESC)
- Wait 1 second for automatic switch back to AUTONOMOUS
- Check logs in spot_base for mode switch messages

### Velocity commands ignored
- Ensure you're in TELEOP mode (check spot_base logs)
- Verify cmd_vel topic is being published: `ros2 topic echo /spot/cmd_vel`
- Check safety limits aren't clamping your commands to zero

## Architecture

```
┌─────────────────────────┐
│ spot_teleop_keyboard    │
│  - Keyboard input       │
│  - Terminal UI          │
│  - Command publishing   │
└───────────┬─────────────┘
            │ ROS2 Topics
            ▼
┌─────────────────────────┐
│ spot_base               │
│  - Mode management      │
│  - Velocity execution   │
│  - Body pose control    │
│  - Navigation (auto)    │
└───────────┬─────────────┘
            │ Boston Dynamics API
            ▼
┌─────────────────────────┐
│ Spot Robot              │
└─────────────────────────┘
```

## Advanced Usage

### Adjusting Safety Limits

Edit the limits in `spot_base.py`:

```python
# In spotMoveBase.__init__():
self.max_height_offset = 0.15  # meters (±15cm)
self.max_pitch_roll = 0.4  # radians (±23°)
self.max_linear_vel = 1.0  # m/s
self.max_angular_vel = 1.5  # rad/s
```

### Changing Default Speed

Edit the default speed in `spot_teleop_keyboard.py`:

```python
# In SpotTeleopKeyboard.__init__():
self.linear_speed = 0.3  # m/s (default speed)
self.angular_speed = 0.5  # rad/s
```

### Customizing Body Control Increments

```python
# In SpotTeleopKeyboard.__init__():
self.height_increment = 0.02  # 2cm per keypress
self.pitch_roll_increment = 0.05  # ~3° per keypress
```

## Logging

The system provides comprehensive logging:

- **INFO**: Mode switches, command executions, speed changes
- **WARN**: Low battery warnings, command timeouts
- **ERROR**: Command failures, API errors

View logs:
```bash
ros2 node list
ros2 topic echo /rosout
```

## Tips

1. **Start slow**: Begin with low speeds (+/- to adjust) until familiar with controls
2. **Use emergency stop**: Press SPACE immediately if something goes wrong
3. **Watch the battery**: Keep an eye on battery percentage in the UI
4. **Reset body pose**: Press HOME to return to default stance if uncomfortable
5. **Mode awareness**: Check logs to confirm mode switches

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review logs from both nodes
3. Verify e-stop status
4. Check network connectivity to Spot

---

**Happy Teleoperating! 🎮🤖**
