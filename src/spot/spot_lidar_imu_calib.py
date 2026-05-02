import time
import math
import bosdyn.client
import bosdyn.geometry
import bosdyn.util
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.api import geometry_pb2, trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
import spot_spot as spot # Your base utility script

class GlobalTrajectoryBuilder:
    def __init__(self):
        self.points = []
        self.total_time = 0.0
        # Start from neutral position
        self.last_pose = (0.0, 0.0, 0.0) # (yaw, roll, pitch)

    def add_segment(self, target_angles, duration, steps=10):
        """
        Smoothly interpolates from the last pose to the target pose.
        """
        s_y, s_r, s_p = self.last_pose
        e_y, e_r, e_p = target_angles
        
        for i in range(1, steps + 1):
            fraction = i / float(steps)
            curr_time = self.total_time + (fraction * duration)
            
            y = s_y + (e_y - s_y) * fraction
            r = s_r + (e_r - s_r) * fraction
            p = s_p + (e_p - s_p) * fraction
            
            orientation = bosdyn.geometry.EulerZXY(yaw=y, roll=r, pitch=p)
            pose = geometry_pb2.SE3Pose(rotation=orientation.to_quaternion())
            
            point = trajectory_pb2.SE3TrajectoryPoint(
                pose=pose, 
                time_since_reference=bosdyn.util.seconds_to_duration(curr_time)
            )
            self.points.append(point)
            
        self.total_time += duration
        self.last_pose = target_angles

    def add_cone_sweep(self, radius, duration, steps=40):
        """
        Performs the conical sweep starting from the current last_pose.
        """
        start_time = self.total_time
        for i in range(1, steps + 1):
            fraction = i / float(steps)
            angle = 2 * math.pi * fraction
            
            # Conical math (Up -> Left -> Down -> Right)
            p = -radius * math.cos(angle)
            r = radius * math.sin(angle)
            y = 0.1 * math.sin(angle)
            
            curr_time = start_time + (fraction * duration)
            orientation = bosdyn.geometry.EulerZXY(yaw=y, roll=r, pitch=p)
            pose = geometry_pb2.SE3Pose(rotation=orientation.to_quaternion())
            
            point = trajectory_pb2.SE3TrajectoryPoint(
                pose=pose, 
                time_since_reference=bosdyn.util.seconds_to_duration(curr_time)
            )
            self.points.append(point)
            
        self.total_time += duration
        self.last_pose = (y, r, p)

def run_seamless_calibration():
    # --- CONFIGURATION ---
    RAD = 0.35
    T_MOVE = 3.0  # Time for transitions
    T_CONE = 6.0 # Time for circle
    T_AXIS = 4.0  # Time for one-way axis stroke
    
    builder = GlobalTrajectoryBuilder()

    print("Building seamless trajectory chain...")

    # Phase 1: Slow Move to UP start point
    builder.add_segment((0, 0, -RAD), T_MOVE)
    builder.add_segment((0, 0, -RAD), T_MOVE)

    # Phase 2: Conical Sweep
    builder.add_cone_sweep(RAD, T_CONE)

    # Phase 3: Single Axis Sweeps (Each move starts where the last one ended)
    # Roll Sweep: Right -> Left -> Neutral
    builder.add_segment((0, RAD, 0), T_MOVE)
    builder.add_segment((0, -RAD, 0), T_AXIS * 2)
    builder.add_segment((0, 0, 0), T_MOVE)

    # Pitch Sweep: Up -> Down -> Neutral
    builder.add_segment((0, 0, -RAD), T_MOVE)
    builder.add_segment((0, 0, RAD), T_AXIS * 2)
    builder.add_segment((0, 0, 0), T_MOVE)

    # Yaw Sweep: Left -> Right -> Neutral
    builder.add_segment((RAD, 0, 0), T_MOVE)
    builder.add_segment((-RAD, 0, 0), T_AXIS * 2)
    builder.add_segment((0, 0, 0), T_MOVE)

    # --- EXECUTION ---
    print(f"Total Trajectory Duration: {builder.total_time:.2f}s")
    
    traj = trajectory_pb2.SE3Trajectory(points=builder.points)
    body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
    params = spot_command_pb2.MobilityParams(body_control=body_control)
    cmd = RobotCommandBuilder.synchro_stand_command(params=params)
    
    # Send the one massive trajectory
    spot.robot_command_client.robot_command(cmd)
    
    # Just one sleep for the entire chain
    time.sleep(builder.total_time + 1.0)

if __name__ == '__main__':
    print("Connecting...")
    while True:
        if spot.connect(): break
        else: time.sleep(1)

    try:
        print("Standing...")
        if not spot.stand(): exit(1)
        time.sleep(1.0)

        run_seamless_calibration()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Finished. Sitting down.")
        spot.sit()