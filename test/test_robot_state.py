import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
import bosdyn.client.math_helpers as math_helpers
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import get_a_tform_b, get_vision_tform_body, get_odom_tform_body

def test():

    hostname = '192.168.50.3'
    
    # The Boston Dynamics Python library uses Python's logging module to
    # generate output. Applications using the library can specify how
    # the logging information should be output.
    bosdyn.client.util.setup_logging(False)

    # The SDK object is the primary entry point to the Boston Dynamics API.
    # create_standard_sdk will initialize an SDK object with typical default
    # parameters. The argument passed in is a string identifying the client.
    sdk = bosdyn.client.create_standard_sdk('CerlabSpotSDK')

    # SpotCAM
    bosdyn.client.spot_cam.register_all_service_clients(sdk)
    
    # A Robot object represents a single robot. Clients using the Boston
    # Dynamics API can manage multiple robots, but this tutorial limits
    # access to just one. The network address of the robot needs to be
    # specified to reach it. This can be done with a DNS name
    # (e.g. spot.intranet.example.com) or an IP literal (e.g. 10.0.63.1)
    robot = sdk.create_robot(hostname)

    # Clients need to authenticate to a robot before being able to use it.
    bosdyn.client.util.authenticate(robot)

    # Establish time sync with the robot. This kicks off a background thread to establish time sync.
    # Time sync is required to issue commands to the robot. After starting time sync thread, block
    # until sync is established.
    robot.time_sync.wait_for_sync()

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, ' \
                                    'such as the estop SDK example, to configure E-Stop.'

    # The robot state client will allow us to get the robot's state information, and construct
    # a command using frame information published by the robot.
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    # Only one client at a time can operate a robot. Clients acquire a lease to
    # indicate that they want to control a robot. Acquiring may fail if another
    # client is currently controlling the robot. When the client is done
    # controlling the robot, it should return the lease so other clients can
    # control it. The LeaseKeepAlive object takes care of acquiring and returning
    # the lease for us.

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    # Setup clients for the robot state and robot command services.
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    # robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=False):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info("Powering on robot... This may take several seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot powered on.")

        # Query the robot for its current state before issuing the stand with yaw command.
        # This state provides a reference pose for issuing a frame based body offset command.
        robot_state = robot_state_client.get_robot_state()
        # print(robot_state)
        frame_tree_snapshot = robot_state.kinematic_state.transforms_snapshot

        # command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        #blocking_stand(command_client, timeout_sec=10)
        #robot.logger.info("Robot standing.")
        # enable_obstacle_avoidance()
        # print("current battery charge status: ", battery_status())

        publish_vision(frame_tree_snapshot)

        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not robot.is_powered_on(), 'Robot power off failed.'
        robot.logger.info('Robot safely powered off.')

def publish_vision(frame_tree_snapshot):
    vision_tform_body = get_vision_tform_body(frame_tree_snapshot)
    print(vision_tform_body)

def publish_odom(frame_tree_snapshot):
    odom_tform_body = get_odom_tform_body(frame_tree_snapshot)
    print(odom_tform_body)



if __name__ == "__main__":
    test()