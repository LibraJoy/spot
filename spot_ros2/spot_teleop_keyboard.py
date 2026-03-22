#!/usr/bin/env python3
"""
Spot Keyboard Teleoperation Node

Controls Spot robot via keyboard with real-time terminal display.

Controls:
  Movement:
    W/S - Forward/Backward
    A/D - Strafe Left/Right
    Q/E - Rotate Left/Right

  Body Control:
    Shift   - Increase height
    Ctrl    - Decrease height
    ↑/↓     - Pitch forward/backward
    ←/→     - Roll left/right
    </>     - Body yaw (rotate body, legs stay in place)
    Home    - Reset body pose to default

  Actions:
    Z - Sit down
    X - Stand up
    Space - Emergency stop
    +/- - Increase/Decrease speed
    ESC - Exit

Author: Claude & User
Date: 2026-03-08
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
import sys
import time
import threading
import math

# Use pynput for keyboard handling (works better than curses for this use case)
try:
    from pynput import keyboard
    from pynput.keyboard import Key
except ImportError:
    print("ERROR: pynput not installed. Install with: pip install pynput")
    sys.exit(1)

# ANSI color codes for terminal
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class SpotTeleopKeyboard(Node):
    def __init__(self):
        super().__init__('spot_teleop_keyboard')

        # Logger
        self.get_logger().info("Initializing Spot Keyboard Teleop Node")

        # Control state
        self.vx = 0.0  # Forward/backward velocity
        self.vy = 0.0  # Left/right velocity
        self.vyaw = 0.0  # Rotational velocity

        # Body pose state
        self.body_height = 0.0  # Height offset
        self.body_roll = 0.0  # Roll angle
        self.body_pitch = 0.0  # Pitch angle
        self.body_yaw = 0.0  # Yaw angle (for body orientation)

        # Speed settings
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.2  # rad/s
        self.speed_increment = 0.05
        self.max_linear_speed = 1.5
        self.max_angular_speed = 1.5

        # Body control increments
        self.height_increment = 0.01  # 1cm per press
        self.pitch_roll_increment = 0.01  # ~0.6 degrees per press
        self.body_yaw_increment = 0.05  # ~3 degrees per press (for fixed position yaw)

        # Safety limits
        self.max_height_offset = 0.25  # ±25cm
        self.max_pitch_roll = 0.4  # ±23 degrees

        # Status data (from subscriptions)
        self.battery_percentage = 0.0
        self.position = [0.0, 0.0, 0.0]
        self.orientation_rpy = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.feedback_msg = ""
        self.last_battery_warning_time = None  # Track last battery warning time

        # Active keys tracking
        self.active_keys = set()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/spot/cmd_vel', 10)
        self.body_pose_pub = self.create_publisher(Twist, '/spot/body_pose', 10)
        self.teleop_active_pub = self.create_publisher(Bool, '/spot/teleop_active', 10)
        self.sit_pub = self.create_publisher(Bool, '/spot/sit', 10)
        self.stand_pub = self.create_publisher(Bool, '/spot/stand', 10)

        # Subscribers
        self.create_subscription(Odometry, '/spot/odom', self.odom_callback, 10)
        self.create_subscription(BatteryState, '/spot/battery', self.battery_callback, 10)
        self.create_subscription(String, '/spot/teleop_feedback', self.feedback_callback, 10)

        # Timers
        self.create_timer(0.1, self.publish_commands)  # 10Hz command publishing
        self.create_timer(1.0, self.publish_heartbeat)  # 1Hz heartbeat
        self.create_timer(0.1, self.update_display)  # 10Hz display update

        # Keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        self.listener.start()

        # Display state
        self.running = True

        self.get_logger().info("Spot Teleop Keyboard Node initialized")
        self.get_logger().info("Press keys to control Spot. ESC to exit.")

        # Clear screen and show initial UI
        self.clear_screen()

    def on_key_press(self, key):
        """Handle key press events"""
        try:
            # Handle character keys
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                self.active_keys.add(char)

                # Movement keys (WASD)
                if char == 'w':
                    self.vx = self.linear_speed
                elif char == 's':
                    self.vx = -self.linear_speed
                elif char == 'a':
                    self.vy = self.linear_speed
                elif char == 'd':
                    self.vy = -self.linear_speed
                elif char == 'q':
                    self.vyaw = self.angular_speed
                elif char == 'e':
                    self.vyaw = -self.angular_speed

                # Action keys
                elif char == 'z':
                    self.sit_pub.publish(Bool(data=True))
                    self.get_logger().info("Sit command sent")
                elif char == 'x':
                    self.stand_pub.publish(Bool(data=True))
                    self.get_logger().info("Stand command sent")

                # Speed adjustment
                elif char == '+' or char == '=':
                    self.increase_speed()
                elif char == '-' or char == '_':
                    self.decrease_speed()

                # Fixed position yaw (body rotation while legs stay in place)
                elif char == '<' or char == ',':
                    self.body_yaw += self.body_yaw_increment
                    self.publish_body_pose()
                    self.get_logger().info(f"Body yaw left: {math.degrees(self.body_yaw):.1f}deg")
                elif char == '>' or char == '.':
                    self.body_yaw -= self.body_yaw_increment
                    self.publish_body_pose()
                    self.get_logger().info(f"Body yaw right: {math.degrees(self.body_yaw):.1f}deg")

            # Handle special keys
            else:
                # Arrow keys for pitch/roll
                if key == Key.up:
                    self.body_pitch = min(self.max_pitch_roll,
                                         self.body_pitch + self.pitch_roll_increment)
                    self.publish_body_pose()
                    self.get_logger().info(f"Pitch up: {math.degrees(self.body_pitch):.1f}deg")

                elif key == Key.down:
                    self.body_pitch = max(-self.max_pitch_roll,
                                         self.body_pitch - self.pitch_roll_increment)
                    self.publish_body_pose()
                    self.get_logger().info(f"Pitch down: {math.degrees(self.body_pitch):.1f}deg")

                elif key == Key.right:
                    self.body_roll = min(self.max_pitch_roll,
                                        self.body_roll + self.pitch_roll_increment)
                    self.publish_body_pose()
                    self.get_logger().info(f"Roll left: {math.degrees(self.body_roll):.1f}deg")

                elif key == Key.left:
                    self.body_roll = max(-self.max_pitch_roll,
                                        self.body_roll - self.pitch_roll_increment)
                    self.publish_body_pose()
                    self.get_logger().info(f"Roll right: {math.degrees(self.body_roll):.1f}deg")

                # Height adjustment
                elif key == Key.shift or key == Key.shift_r:
                    self.body_height = min(self.max_height_offset,
                                          self.body_height + self.height_increment)
                    self.publish_body_pose()
                    self.get_logger().info(f"Height up: {self.body_height*100:.1f}cm")

                elif key == Key.ctrl or key == Key.ctrl_r or key == Key.ctrl_l:
                    self.body_height = max(-self.max_height_offset,
                                          self.body_height - self.height_increment)
                    self.publish_body_pose()
                    self.get_logger().info(f"Height down: {self.body_height*100:.1f}cm")

                # Reset body pose
                elif key == Key.home:
                    self.reset_body_pose()

                # Emergency stop
                elif key == Key.space:
                    self.emergency_stop()

                # Exit - only quit, don't emergency stop
                elif key == Key.esc:
                    self.get_logger().info("ESC pressed - shutting down teleop")
                    self.running = False
                    return False

        except Exception as e:
            self.get_logger().error(f"Key press error: {e}")

    def on_key_release(self, key):
        """Handle key release events"""
        try:
            # Stop movement when keys released
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                if char in self.active_keys:
                    self.active_keys.remove(char)

                if char in ['w', 's']:
                    self.vx = 0.0
                elif char in ['a', 'd']:
                    self.vy = 0.0
                elif char in ['q', 'e']:
                    self.vyaw = 0.0

        except Exception as e:
            self.get_logger().error(f"Key release error: {e}")

    def publish_commands(self):
        """Publish velocity commands at 10Hz"""
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.vyaw

        self.cmd_vel_pub.publish(twist)

    def publish_body_pose(self):
        """Publish body pose command"""
        # Using Twist message: linear.z=height, angular.x=roll, y=pitch, z=yaw
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = self.body_height
        twist.angular.x = self.body_roll
        twist.angular.y = self.body_pitch
        twist.angular.z = self.body_yaw

        self.body_pose_pub.publish(twist)

    def publish_heartbeat(self):
        """Publish teleop active heartbeat at 1Hz"""
        self.teleop_active_pub.publish(Bool(data=True))

    def reset_body_pose(self):
        """Reset body pose to default"""
        self.body_height = 0.0
        self.body_roll = 0.0
        self.body_pitch = 0.0
        self.body_yaw = 0.0
        self.publish_body_pose()
        self.get_logger().info("Body pose reset to default")

    def emergency_stop(self):
        """Emergency stop - zero all velocities"""
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.active_keys.clear()

        # Publish zero velocity
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        self.get_logger().warn("EMERGENCY STOP")

    def increase_speed(self):
        """Increase movement speed"""
        self.linear_speed = min(self.max_linear_speed,
                               self.linear_speed + self.speed_increment)
        self.angular_speed = min(self.max_angular_speed,
                                self.angular_speed + self.speed_increment)
        self.get_logger().info(f"Speed increased: {self.linear_speed:.2f} m/s")

    def decrease_speed(self):
        """Decrease movement speed"""
        self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
        self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
        self.get_logger().info(f"Speed decreased: {self.linear_speed:.2f} m/s")

    def odom_callback(self, msg):
        """Update position and orientation from odometry"""
        self.position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]

        # Convert quaternion to roll, pitch, yaw
        q = msg.pose.pose.orientation
        self.orientation_rpy = self.quaternion_to_euler(q.x, q.y, q.z, q.w)

    def battery_callback(self, msg):
        """Update battery status"""
        self.battery_percentage = msg.percentage * 100.0

    def feedback_callback(self, msg):
        """Update feedback message"""
        self.feedback_msg = msg.data

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def clear_screen(self):
        """Clear terminal screen"""
        print("\033[2J\033[H", end='')

    def update_display(self):
        """Update terminal display at 10Hz"""
        if not self.running:
            return

        # Move cursor to top
        print("\033[H", end='')

        # Header
        print(f"{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.ENDC}")
        print(f"{Colors.BOLD}{Colors.CYAN}{'SPOT KEYBOARD TELEOP':^70}{Colors.ENDC}")
        print(f"{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.ENDC}\n")

        # Status section
        print(f"{Colors.BOLD}STATUS:{Colors.ENDC}")

        # Battery with color coding
        battery_color = Colors.GREEN if self.battery_percentage > 50 else \
                       Colors.YELLOW if self.battery_percentage > 20 else Colors.RED
        print(f"  Battery:     {battery_color}{self.battery_percentage:5.1f}%{Colors.ENDC}")

        # Position
        print(f"  Position:    X:{self.position[0]:6.2f}m  Y:{self.position[1]:6.2f}m  Z:{self.position[2]:6.2f}m")

        # Orientation
        roll_deg = math.degrees(self.orientation_rpy[0])
        pitch_deg = math.degrees(self.orientation_rpy[1])
        yaw_deg = math.degrees(self.orientation_rpy[2])
        print(f"  Orientation: R:{roll_deg:6.1f}deg  P:{pitch_deg:6.1f}deg  Y:{yaw_deg:6.1f}deg")

        print()

        # Control section
        print(f"{Colors.BOLD}CONTROL:{Colors.ENDC}")

        # Velocity
        vel_color = Colors.GREEN if (abs(self.vx) > 0.01 or abs(self.vy) > 0.01 or abs(self.vyaw) > 0.01) else Colors.ENDC
        print(f"  {vel_color}Velocity:    Fwd:{self.vx:5.2f} m/s  Left:{self.vy:5.2f} m/s  Rot:{self.vyaw:5.2f} rad/s{Colors.ENDC}")

        # Speed setting
        print(f"  Speed:       {self.linear_speed:.2f} m/s (+/-: adjust)")

        print()

        # Body pose
        print(f"{Colors.BOLD}BODY POSE:{Colors.ENDC}")
        body_roll_deg = math.degrees(self.body_roll)
        body_pitch_deg = math.degrees(self.body_pitch)
        body_yaw_deg = math.degrees(self.body_yaw)
        print(f"  Height:      {self.body_height*100:5.1f} cm  (Shift/Ctrl)")
        print(f"  Roll:        {body_roll_deg:6.1f}deg (Left/Right arrows)")
        print(f"  Pitch:       {body_pitch_deg:6.1f}deg (Up/Down arrows)")
        print(f"  Body Yaw:    {body_yaw_deg:6.1f}deg (</> keys)")

        print()

        # Feedback
        if self.feedback_msg:
            print(f"{Colors.BOLD}FEEDBACK:{Colors.ENDC}")
            print(f"  {Colors.YELLOW}{self.feedback_msg}{Colors.ENDC}")
            print()

        # Controls
        print(f"{Colors.BOLD}CONTROLS:{Colors.ENDC}")
        print(f"  {Colors.CYAN}Movement:{Colors.ENDC}  W/S:Fwd/Back  A/D:Left/Right  Q/E:Rotate")
        print(f"  {Colors.CYAN}Body:    {Colors.ENDC}  Shift/Ctrl:Height  Arrows:Pitch/Roll  </>:BodyYaw  Home:Reset")
        print(f"  {Colors.CYAN}Actions: {Colors.ENDC}  Z:Sit  X:Stand  Space:E-STOP  +/-:Speed  ESC:Exit")

        print(f"\n{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.ENDC}")

        # Pad with empty lines to prevent scrolling
        for _ in range(5):
            print()

    def shutdown(self):
        """Cleanup on shutdown"""
        self.get_logger().info("Shutting down teleop keyboard node")
        self.running = False
        # Zero velocities but don't call emergency_stop to avoid extra logging
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.listener.stop()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SpotTeleopKeyboard()

        # Spin in a separate thread
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        # Wait for node to signal shutdown
        while node.running:
            time.sleep(0.1)

        node.shutdown()
        node.destroy_node()

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        print("\n\nSpot Teleop Keyboard shutdown complete")


if __name__ == '__main__':
    main()
