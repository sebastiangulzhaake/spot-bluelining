import sys
import time
from xmlrpc.client import boolean

import bosdyn.client
import bosdyn.client.lease
from bosdyn.client.sdk import create_standard_sdk
import bosdyn.client.util
from bosdyn.api import world_object_pb2
from bosdyn.client.math_helpers import Quat
from bosdyn.client.frame_helpers import (GRAV_ALIGNED_BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_vision_tform_body)
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, block_for_trajectory_cmd,
                                         block_until_arm_arrives, blocking_stand)

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_id import RobotIdClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient

from controller_thread import ControllerThread
from robot_state_monitor import RobotStateMonitor
from robot_state_thread import RobotStateThread
from data_stream_monitor import DataStreamMonitor
from data_stream_thread import DataStreamThread
from input_monitor import InputMonitor
from input_thread import InputThread

from scipy.spatial.transform import Rotation as R

import numpy as np

class BlueliningProgram():
    def __init__(self, robot, options):
        self._robot = robot
        self._config = options
        self._robot_id = robot.ensure_client(RobotIdClient.default_service_name).get_id(timeout=0.4)
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)

        self._data_stream_monitor = None
        self._data_stream_thread = None
        self._robot_state_monitor = None
        self._robot_state_thread = None
        self._controller_thread = None
        self._input_monitor = None
        self._input_thread = None

        self._fiducial_rt_world = None

        # Arbitrary goal points
        self._goalpoints = np.array([[3000,-3000],[3000,-2000],[0,-4000],[0,-4500],[0,-5000]])

    def power_off(self):
        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        self._robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not self._robot.is_powered_on(), "Robot power off failed."
        self._robot.logger.info("Robot safely powered off.")
        return

    def start(self):
        """A program for bluelining a set of points in an automated sequence."""
        self._robot.time_sync.wait_for_sync()

        # Create data stream monitor and start data stream thread
        # Comment out the lines for the tracker that is not used

        #Krypton
        host='192.168.18.89' #ip of krypton Computer (server)
        port=12000 #same as server
        tracker = 'krypton'

        #Leica
        # host='192.168.78.31' #ip of this Computer (Leica)
        # port=65431 #same as server
        # tracker = 'leica'

        self._data_stream_monitor = DataStreamMonitor()

        self._data_stream_thread = DataStreamThread(self._data_stream_monitor, host, port, tracker)
        self._data_stream_thread.start()
 
        # Initiate RobotStateMonitor and set the first robot state
        self._robot_state_monitor = RobotStateMonitor()
        robot_state = self._robot_state_client.get_robot_state()
        self._robot_state_monitor.set_robot_state(robot_state)

        # Initiate RobotStateThread and start it
        self._robot_state_thread = RobotStateThread(self._robot_state_client, self._robot_state_monitor)
        self._robot_state_thread.start()

        # Inititate InputMonitor
        self._input_monitor = InputMonitor()

        # Inititate ControllerThread and start it
        self._controller_thread = ControllerThread(self._robot, self._robot_command_client, self._data_stream_monitor, self._robot_state_monitor, self._goalpoints, self._input_monitor)
        self._controller_thread.start()

        # Check cyclicaly for input to exit program and power off robot
        self._robot.logger.info("Press 'p' to power off robot.")

        while True:
            command = input()
            self._input_monitor.set_input(command)
            if command == 'p':  # if key 'p' is pressed followed by enter

                self._controller_thread.stop()
                self._controller_thread.join()
                self._robot.logger.info("ControllerThread stopped. ")

                self._robot_state_thread.stop()
                self._robot_state_thread.join()
                self._robot.logger.info("RobotStateThread stopped")

                self._data_stream_thread.stop()
                self._data_stream_thread.join()
                self._robot.logger.info("DataStreamThread stopped. ")

                self.power_off()
                return

def main(argv):
    """Command-line interface."""
    import argparse

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)

    # See hello_spot.py for an explanation of these lines.
    bosdyn.client.util.setup_logging(options.verbose)
    # Create robot object.
    sdk = create_standard_sdk('Blueling')
    robot = sdk.create_robot(options.hostname)
    try:
        bosdyn.client.util.authenticate(robot)
        robot.start_time_sync()

        # Verify the robot has an arm
        assert robot.has_arm(), "Robot requires an arm to run this example."

        # Verify the robot is not estopped and that an external application has registered and holds
        # an estop endpoint.
        assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                        "such as the estop SDK example, to configure E-Stop."

        lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

        with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            # Initiate and start BlueliningProgram
            bluelining_program = BlueliningProgram(robot, options)
            bluelining_program.start()

    except Exception as exc: 
        logger = bosdyn.client.util.get_logger()
        logger.exception("Threw an exception")
        return False     

if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)