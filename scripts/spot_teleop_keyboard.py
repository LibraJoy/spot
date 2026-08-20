#!/usr/bin/env python3
"""
Spot Keyboard Teleoperation Node (ROS1) — SSH compatible

Uses termios/stdin (no X11 required), works over SSH.
Highest-priority control: suppresses the autonomous planner
(/spot/waypoint) and RViz (/move_base_simple/goal) while alive.

Multi-key control: each movement key tracks its own "last pressed"
timestamp. Terminal key-repeat keeps the timestamp fresh while a key
is held; stopping a key causes it to expire after KEY_HOLD_TIMEOUT.
Multiple keys active simultaneously → combined velocity.

Controls:
  Movement (combinable):
    W/S   - Forward/Backward
    A/D   - Strafe Left/Right
    Q/E   - Rotate Left/Right

  Body Control (incremental per press):
    R/F        - Height up/down  (limit ±10 cm)
    Up/Down    - Pitch forward/backward
    Left/Right - Roll left/right
    ,/.        - Body yaw left/right  (limit ±25°)
    H          - Reset body pose to default

  Actions:
    Z          - Sit
    X          - Stand
    Space      - Emergency stop
    +/-        - Increase/Decrease speed
    ESC / Ctrl+C - Exit
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
import sys
import tty
import termios
import select
import time
import threading
import math

# ── terminal escape sequences ────────────────────────────────────────
# KEY_UP     = '\x1b[A'
# KEY_DOWN   = '\x1b[B'
# KEY_RIGHT  = '\x1b[C'
# KEY_LEFT   = '\x1b[D'
KEY_UP     = 'i'
KEY_DOWN   = 'k'
KEY_RIGHT  = 'l'
KEY_LEFT   = 'j'
KEY_ESC    = '\x1b'
KEY_CTRL_C = '\x03'
KEY_SWITCH_OUT = 'c'
KEY_SWITCH_BACK = 'v'

MOVE_KEYS = {'w', 's', 'a', 'd', 'q', 'e'}

# How long a movement key stays "active" without a fresh keypress.
# Must be longer than terminal key-repeat interval (~30 ms) but short
# enough to feel responsive. 250 ms works well even over SSH.
KEY_HOLD_TIMEOUT = 0.25


def get_key(settings, timeout=0.1):
    """Read one keypress (or escape sequence) with timeout. Returns '' on timeout."""
    tty.setraw(sys.stdin.fileno())
    try:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return ''
        key = sys.stdin.read(1)
        if key == '\x1b':
            # Give SSH up to 150 ms to deliver the rest of the sequence.
            # A bare ESC with nothing following within that window is a true ESC.
            rlist2, _, _ = select.select([sys.stdin], [], [], 0.15)
            if rlist2:
                key += sys.stdin.read(1)          # '['
                rlist3, _, _ = select.select([sys.stdin], [], [], 0.15)
                if rlist3:
                    key += sys.stdin.read(1)      # 'A' / 'B' / 'C' / 'D' / …
        return key
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class Colors:
    CYAN   = '\033[96m'
    GREEN  = '\033[92m'
    YELLOW = '\033[93m'
    ENDC   = '\033[0m'
    BOLD   = '\033[1m'


class SpotTeleopKeyboard:
    def __init__(self):
        # ── velocity state (computed from active_keys) ────────────────
        self.vx    = 0.0
        self.vy    = 0.0
        self.vyaw  = 0.0
        self._was_moving = False

        # Per-key timestamps for simultaneous-key support
        self.active_keys = {}  # key -> last_press_time (movement keys only)

        # ── body pose state ───────────────────────────────────────────
        self.body_height = 0.0
        self.body_roll   = 0.0
        self.body_pitch  = 0.0
        self.body_yaw    = 0.0

        # ── speed settings ────────────────────────────────────────────
        self.linear_speed    = 0.3
        self.angular_speed   = 0.2
        self.speed_increment = 0.05
        self.max_linear_speed  = 1.5
        self.max_angular_speed = 1.5

        # ── body control increments and limits ────────────────────────
        self.height_increment     = 0.01          # 1 cm per press
        self.pitch_roll_increment = 0.1          # 0.01~0.6° per press
        self.body_yaw_increment   = 0.1          # 0.05~3° per press

        self.max_height_offset = 0.10             # ±10 cm
        self.max_pitch_roll    = 0.40             # ±23°
        self.max_body_yaw      = math.radians(25) # ±25°

        # ── status data ───────────────────────────────────────────────
        self.position        = [0.0, 0.0, 0.0]
        self.orientation_rpy = [0.0, 0.0, 0.0]
        self.feedback_msg    = ""

        # ── publishers ────────────────────────────────────────────────
        self.cmd_vel_pub       = rospy.Publisher('/spot/cmd_vel',       Twist, queue_size=10)
        self.body_pose_pub     = rospy.Publisher('/spot/body_pose',     Twist, queue_size=10)
        self.teleop_active_pub = rospy.Publisher('/spot/teleop_active', Bool,  queue_size=10)
        self.sit_pub           = rospy.Publisher('/spot/sit',           Bool,  queue_size=10)
        self.stand_pub         = rospy.Publisher('/spot/stand',         Bool,  queue_size=10)

        # ── subscribers ───────────────────────────────────────────────
        rospy.Subscriber('/spot/odom',            Odometry, self.odom_callback)
        rospy.Subscriber('/spot/teleop_feedback', String,   self.feedback_callback)

        # ── timers ────────────────────────────────────────────────────active_keys
        rospy.Timer(rospy.Duration(0.1), self.publish_commands)   # 10 Hz

        self.running = True
        # self.teleop_mode = True
        # rospy.loginfo("Spot Teleop Keyboard ready — ESC or Ctrl+C to exit, c to switch out, v to switch back.")

    # ── velocity from active keys ─────────────────────────────────────

    def _update_velocity(self):
        """Recompute vx/vy/vyaw from still-active movement keys."""
        now = time.time()
        # Expire keys that haven't been pressed recently
        self.active_keys = {k: t for k, t in self.active_keys.items()
                            if now - t <= KEY_HOLD_TIMEOUT}

        vx = vy = vyaw = 0.0
        for k in self.active_keys:
            if   k == 'w': vx   += self.linear_speed
            elif k == 's': vx   -= self.linear_speed
            elif k == 'a': vy   += self.linear_speed
            elif k == 'd': vy   -= self.linear_speed
            elif k == 'q': vyaw += self.angular_speed
            elif k == 'e': vyaw -= self.angular_speed
        self.vx, self.vy, self.vyaw = vx, vy, vyaw

    # ── key handling ─────────────────────────────────────────────────

    def handle_key(self, key):
        """Process one key (or '' on timeout) from the main loop."""
        # Always refresh velocity — prunes any expired keys
        if key != '':
            k = key.lower() if len(key) == 1 else key
            self._process_key(k, key)
        self._update_velocity()

    def _process_key(self, k, raw_key):
        # ── movement keys ─────────────────────────────────────────────
        if k in MOVE_KEYS:
            self.active_keys[k] = time.time()
            return  # velocity recomputed in handle_key after this

        # ── body pose ─────────────────────────────────────────────────
        if raw_key == KEY_UP:
            self.body_pitch = min(self.max_pitch_roll,
                                  self.body_pitch + self.pitch_roll_increment)
            self.publish_body_pose()
        elif raw_key == KEY_DOWN:
            self.body_pitch = max(-self.max_pitch_roll,
                                  self.body_pitch - self.pitch_roll_increment)
            self.publish_body_pose()
        elif raw_key == KEY_RIGHT:
            self.body_roll = min(self.max_pitch_roll,
                                 self.body_roll + self.pitch_roll_increment)
            self.publish_body_pose()
        elif raw_key == KEY_LEFT:
            self.body_roll = max(-self.max_pitch_roll,
                                 self.body_roll - self.pitch_roll_increment)
            self.publish_body_pose()
        elif k == 'r':
            self.body_height = min(self.max_height_offset,
                                   self.body_height + self.height_increment)
            self.publish_body_pose()
        elif k == 'f':
            self.body_height = max(-self.max_height_offset,
                                   self.body_height - self.height_increment)
            self.publish_body_pose()
        elif k in (',', '<'):
            self.body_yaw = min(self.max_body_yaw,
                                self.body_yaw + self.body_yaw_increment)
            self.publish_body_pose()
        elif k in ('.', '>'):
            self.body_yaw = max(-self.max_body_yaw,
                                self.body_yaw - self.body_yaw_increment)
            self.publish_body_pose()
        elif k == 'h':
            self.reset_body_pose()

        # ── actions ───────────────────────────────────────────────────
        elif k == 'z':
            self.sit_pub.publish(Bool(True))
            rospy.loginfo("Sit command sent")
        elif k == 'x':
            self.stand_pub.publish(Bool(True))
            rospy.loginfo("Stand command sent")
        elif raw_key == ' ':
            self.emergency_stop()
        elif k in ('+', '='):
            self.increase_speed()
        elif k in ('-', '_'):
            self.decrease_speed()

        # ── exit ──────────────────────────────────────────────────────
        elif raw_key in (KEY_ESC, KEY_CTRL_C):
            rospy.loginfo("Exit key — shutting down teleop")
            self.running = False
        
        # ── switch ──────────────────────────────────────────────────────
        # elif raw_key == KEY_SWITCH_OUT:
        #     if self.teleop_mode:
        #         self.teleop_mode = False
        #         rospy.loginfo("Switched OUT of teleop mode — cmd_vel will no longer be published")
        #     else:
        #         rospy.loginfo("Already switched out of teleop mode")
        # elif raw_key == KEY_SWITCH_BACK:
        #     if not self.teleop_mode:
        #         self.teleop_mode = True
        #         rospy.loginfo("Switched BACK INTO teleop mode — cmd_vel will be published again")
        #     else:
        #         rospy.loginfo("Already in teleop mode")

    # ── publishers ────────────────────────────────────────────────────

    def publish_commands(self, event):
        """10 Hz: publish cmd_vel while moving or on the one stop frame; track teleop_active state."""
        self._update_velocity()  # keep expiry running even when no key is typed
        is_moving = abs(self.vx) > 0.001 or abs(self.vy) > 0.001 or abs(self.vyaw) > 0.001
        if is_moving or self._was_moving:
            twist = Twist()
            twist.linear.x  = self.vx
            twist.linear.y  = self.vy
            twist.angular.z = self.vyaw
            self.cmd_vel_pub.publish(twist)

        # Publish True while keys are held (heartbeat); False on the first tick after release.
        if is_moving:
            self.teleop_active_pub.publish(Bool(True))
        elif self._was_moving:
            self.teleop_active_pub.publish(Bool(False))

        self._was_moving = is_moving

    def publish_body_pose(self):
        twist = Twist()
        twist.linear.z  = self.body_height
        twist.angular.x = self.body_roll
        twist.angular.y = self.body_pitch
        twist.angular.z = self.body_yaw
        self.body_pose_pub.publish(twist)

    # ── controls ─────────────────────────────────────────────────────

    def reset_body_pose(self):
        self.body_height = self.body_roll = self.body_pitch = self.body_yaw = 0.0
        self.publish_body_pose()
        rospy.loginfo("Body pose reset")

    def emergency_stop(self):
        self.active_keys.clear()
        self.vx = self.vy = self.vyaw = 0.0
        self._was_moving = True   # send one stop Twist on next publish tick
        rospy.logwarn("EMERGENCY STOP")

    def increase_speed(self):
        self.linear_speed  = min(self.max_linear_speed,
                                 self.linear_speed  + self.speed_increment)
        self.angular_speed = min(self.max_angular_speed,
                                 self.angular_speed + self.speed_increment)
        rospy.loginfo("Speed: %.2f m/s", self.linear_speed)

    def decrease_speed(self):
        self.linear_speed  = max(0.1, self.linear_speed  - self.speed_increment)
        self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
        rospy.loginfo("Speed: %.2f m/s", self.linear_speed)

    # ── subscribers ───────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.position = [msg.pose.pose.position.x,
                         msg.pose.pose.position.y,
                         msg.pose.pose.position.z]
        q = msg.pose.pose.orientation
        self.orientation_rpy = self._quat_to_euler(q.x, q.y, q.z, q.w)

    def feedback_callback(self, msg):
        self.feedback_msg = msg.data

    def _quat_to_euler(self, x, y, z, w):
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        sinp  = 2*(w*y - z*x)
        pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return [roll, pitch, yaw]

    # ── display (called from main loop — same thread as get_key) ─────

    def print_display(self):
        print("\033[H", end='')
        B, C, E = Colors.BOLD, Colors.CYAN, Colors.ENDC

        print(f"{B}{C}{'='*62}{E}")
        print(f"{B}{C}{'SPOT KEYBOARD TELEOP':^62}{E}")
        print(f"{B}{C}{'='*62}{E}\n")

        print(f"{B}POSITION:{E}  "
              f"X:{self.position[0]:6.2f}m  "
              f"Y:{self.position[1]:6.2f}m  "
              f"Z:{self.position[2]:6.2f}m")
        rd, pd, yd = (math.degrees(v) for v in self.orientation_rpy)
        print(f"{B}ORIENT:  {E}  R:{rd:6.1f}°  P:{pd:6.1f}°  Y:{yd:6.1f}°\n")

        moving = abs(self.vx)>0.01 or abs(self.vy)>0.01 or abs(self.vyaw)>0.01
        vc = Colors.GREEN if moving else E
        active = ' '.join(sorted(self.active_keys)) or '—'
        print(f"{B}VELOCITY:{E}  {vc}Fwd:{self.vx:5.2f}  "
              f"Left:{self.vy:5.2f}  Rot:{self.vyaw:5.2f}{E}  "
              f"active:[{active}]")
        print(f"{B}SPEED:   {E}  {self.linear_speed:.2f} m/s  (+/- adjust)\n")

        print(f"{B}BODY POSE:{E}")
        print(f"  Height {self.body_height*100:+5.1f}cm / ±{self.max_height_offset*100:.0f}cm  "
              f"Yaw {math.degrees(self.body_yaw):+5.1f}° / ±{math.degrees(self.max_body_yaw):.0f}°")
        print(f"  Roll   {math.degrees(self.body_roll):+5.1f}°  "
              f"Pitch {math.degrees(self.body_pitch):+5.1f}°\n")

        if self.feedback_msg:
            print(f"{B}FEEDBACK:{E}  {Colors.YELLOW}{self.feedback_msg}{E}\n")

        print(f"{B}KEYS:{E}")
        print(f"  {C}Move (combinable):{E} W/S Fwd/Back  A/D Strafe  Q/E Rotate")
        print(f"  {C}Body:{E}              R/F Height  IJKL Pitch/Roll  ,/. Yaw  H Reset")
        print(f"  {C}Action:{E}            Z Sit  X Stand  Space E-STOP  +/- Speed  ESC Exit")
        print(f"\n{B}{C}{'='*62}{E}")
        for _ in range(3):
            print(' ' * 62)

    def shutdown(self):
        rospy.loginfo("Teleop shutdown — zeroing velocity")
        self.active_keys.clear()
        self.vx = self.vy = self.vyaw = 0.0
        self.cmd_vel_pub.publish(Twist())
        self.teleop_active_pub.publish(Bool(False))


# ── entry point ───────────────────────────────────────────────────────

def main():
    rospy.init_node('spot_teleop_keyboard', anonymous=True)

    settings = termios.tcgetattr(sys.stdin)
    node = SpotTeleopKeyboard()

    spin_thread = threading.Thread(target=rospy.spin, daemon=True)
    spin_thread.start()

    print("\033[2J\033[H", end='')  # clear screen once

    last_display = 0.0
    try:
        while node.running and not rospy.is_shutdown():
            key = get_key(settings, timeout=0.1)
            node.handle_key(key)

            now = time.time()
            if now - last_display >= 0.1:
                node.print_display()
                last_display = now

    except Exception as e:
        rospy.logerr("Teleop error: %s", e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.shutdown()
        print("\nSpot Teleop shutdown complete\n")


if __name__ == '__main__':
    main()
