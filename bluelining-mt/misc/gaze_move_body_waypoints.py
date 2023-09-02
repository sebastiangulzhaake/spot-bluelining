# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Test to try and lock gaze and then align body with gaze.
"""
import argparse
from math import pi
import sys

import numpy as np

from google.protobuf import duration_pb2
import time

import bosdyn.client
from bosdyn.client import robot_command
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import (geometry_pb2, trajectory_pb2, arm_command_pb2, synchronized_command_pb2, robot_command_pb2,
                        mobility_command_pb2, basic_command_pb2)
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b, get_se2_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, block_for_trajectory_cmd,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient


def gaze_move_body_waypoints(config):
    """Test to try and lock gaze and then align body with gaze.
    """

    # See hello_spot.py for an explanation of these lines.
    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('GazeMoveBodyWaypoints')
    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    assert robot.has_arm(), "Robot requires an arm to run this example."

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                    "such as the estop SDK example, to configure E-Stop."

    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info("Powering on robot... This may take a several seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot powered on.")

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # SpotCommandHelper for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        robot.logger.info("Commanding robot to stand...")
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)
        robot.logger.info("Robot standing.")

        # Unstow the arm
        unstow = RobotCommandBuilder.arm_ready_command()

        # Issue the command via the RobotCommandClient
        unstow_command_id = command_client.robot_command(unstow)
        robot.logger.info("Unstow command issued.")

        block_until_arm_arrives(command_client, unstow_command_id, 3.0)

        # Convert the location from the moving base frame to the world frame.
        robot_state = robot_state_client.get_robot_state()
        vision_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         VISION_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        # Look at a point 2 meters in front and 1 meters to the left.
        # We are not specifying a hand location, the robot will pick one.
        gaze_target_in_vision = vision_T_flat_body.transform_point(x=2.0, y=1.0, z=0.3)

        # Start with simple gaze
        gaze_command = RobotCommandBuilder.arm_gaze_command(gaze_target_in_vision[0],
                                                            gaze_target_in_vision[1],
                                                            gaze_target_in_vision[2],
                                                            VISION_FRAME_NAME)

        # Issue the command via the RobotCommandClient
        gaze_command_id = command_client.robot_command(gaze_command)
        robot.logger.info("Gaze command issued.")

        block_until_arm_arrives(command_client, gaze_command_id, 3.0)



        # Get current robot state.
        robot_state = robot_state_client.get_robot_state()
        vision_T_body = get_se2_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         VISION_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)


        waypoint_traj_time = 5

        duration_seconds = int(waypoint_traj_time)
        duration_nanos = int((waypoint_traj_time - duration_seconds) * 1e9)

        # Matrix with waypoints: col1 = x, col2 = y, col3 = heading
        waypoints = np.array([[1, 0, 0.375*pi], [2, 0, 0.5*pi], [3, 0, 0.625*pi], [4, 0, 0.75*pi]])
        rows, cols = waypoints.shape
        traj_points = []
        i = 0
        for point in waypoints:
            body_pose_in_body = math_helpers.SE2Pose(x=point[0], y=point[1], angle=point[2])
            body_pose_in_vision = vision_T_body * body_pose_in_body

            position = geometry_pb2.Vec2(x=body_pose_in_vision.x, y=body_pose_in_vision.y)
            pose = geometry_pb2.SE2Pose(position=position, angle=body_pose_in_vision.angle)
            point = trajectory_pb2.SE2TrajectoryPoint(pose=pose, time_since_reference=duration_pb2.Duration(seconds=duration_seconds + i,
                                                       nanos=duration_nanos))

            traj_points.append(point)
            i+=5

        traj = trajectory_pb2.SE2Trajectory(points=[traj_points[0], traj_points[1], traj_points[2], traj_points[3]], interpolation=2)
        traj_command = basic_command_pb2.SE2TrajectoryCommand.Request(trajectory=traj,
                                                                      se2_frame_name=VISION_FRAME_NAME)
        mobility_command = mobility_command_pb2.MobilityCommand.Request(
            se2_trajectory_request=traj_command)
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            mobility_command=mobility_command)
        robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

        synchro_command = RobotCommandBuilder.build_synchro_command(robot_cmd, gaze_command)

        end_time=20.0
        
        cmd_id = command_client.robot_command(command=synchro_command,
                                            end_time_secs=time.time() + end_time)

        #block_for_trajectory_cmd(command_client, cmd_id, timeout_sec=time.time() + end_time)       

        robot.logger.info("Press 'P' to power off robot.")

        while True:  # making a loop
            
            command = input()
            if command == 'p':  # if key 'p' is pressed followed by enter
            
                # Power the robot off. By specifying "cut_immediately=False", a safe power off command
                # is issued to the robot. This will attempt to sit the robot before powering off.
                robot.power_off(cut_immediately=False, timeout_sec=20)
                assert not robot.is_powered_on(), "Robot power off failed."
                robot.logger.info("Robot safely powered off.")
                return


def main(argv):
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)
    try:
        gaze_move_body_waypoints(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception("Threw an exception")
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
