# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Test to try and lock gaze while moving the arm to a new position and following with body.
"""
import argparse
import sys

from google.protobuf import duration_pb2

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import (geometry_pb2, trajectory_pb2, arm_command_pb2, synchronized_command_pb2, robot_command_pb2,
                        mobility_command_pb2, basic_command_pb2)
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b, get_se2_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, block_for_trajectory_cmd,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient


def gaze_lock_body_follow(config):
    """Test to try and lock gaze while moving the arm to a new position and asks the body to move to a good position based
       on the arm's new location."""

    # See hello_spot.py for an explanation of these lines.
    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('GazeLockBodyFollowClient')
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
        gaze_target_in_vision = vision_T_flat_body.transform_point(x=2.0, y=1.0, z=0)

        # Start with simple gaze
        gaze_command = RobotCommandBuilder.arm_gaze_command(gaze_target_in_vision[0],
                                                            gaze_target_in_vision[1],
                                                            gaze_target_in_vision[2],
                                                            VISION_FRAME_NAME)

        # Issue the command via the RobotCommandClient
        gaze_command_id = command_client.robot_command(gaze_command)
        robot.logger.info("Gaze command issued.")

        block_until_arm_arrives(command_client, gaze_command_id, 3.0)

        body_traj_time = 5

        duration_seconds = int(body_traj_time)
        duration_nanos = int((body_traj_time - duration_seconds) * 1e9)

        # Get current robot state.
        robot_state = robot_state_client.get_robot_state()
        vision_T_hand = get_se2_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         VISION_FRAME_NAME, HAND_FRAME_NAME)

        # Wanted coordinates for body expressed in hand's frame
        body_x = -0.75
        body_y = 0
            
        # We specify an orientation for the body.
        angle = 0

        body_vec2 = geometry_pb2.Vec2(x=body_x, y=body_y)

        body_pose_in_hand = geometry_pb2.SE2Pose(position=body_vec2, angle=angle)

        body_pose_in_vision = vision_T_hand * math_helpers.SE2Pose.from_proto(
            body_pose_in_hand)

        goal_point = trajectory_pb2.SE2TrajectoryPoint(pose=body_pose_in_vision.to_proto(),
                                                        time_since_reference=duration_pb2.Duration(seconds=duration_seconds,
                                                       nanos=duration_nanos))
        
        body_traj = trajectory_pb2.SE2Trajectory(points=[goal_point])

        traj_command = basic_command_pb2.SE2TrajectoryCommand.Request(se2_frame_name=VISION_FRAME_NAME,
                                                                        trajectory=body_traj)

        #follow_arm_request=basic_command_pb2.FollowArmCommand.Request(body_offset_from_hand=offset))


        mobility_command = mobility_command_pb2.MobilityCommand.Request(se2_trajectory_request=traj_command)
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(mobility_command=mobility_command)
        body_command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

        synchro_command = RobotCommandBuilder.build_synchro_command(body_command)

        body_command_id = command_client.robot_command(synchro_command)
        robot.logger.info("Move body command issued")

        block_for_trajectory_cmd(command_client, body_command_id, timeout_sec=5.0)       



        # Convert the location from the moving base frame to the world frame.
        robot_state = robot_state_client.get_robot_state()
        vision_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         VISION_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        # Look at a point 2 meters in front and 1 meters to the left.
        # We are not specifying a hand location, the robot will pick one.
        gaze_target_in_vision = vision_T_flat_body.transform_point(x=2.0, y=1.0, z=0)      

        pos = geometry_pb2.Vec3(x=gaze_target_in_vision[0], y=gaze_target_in_vision[1], z=gaze_target_in_vision[2])
        point1 = trajectory_pb2.Vec3TrajectoryPoint(point=pos)

        gaze_traj = trajectory_pb2.Vec3Trajectory(points=[point1])

        # Now make a gaze trajectory that moves the hand to a new point while maintaining the gaze.
        # We'll use the same trajectory as before, but add a trajectory for the hand to move to.

        hand_traj_time = 15  # take 20 s to complete trajectory.

        duration_seconds = int(hand_traj_time)
        duration_nanos = int((hand_traj_time - duration_seconds) * 1e9)

        # Hand will start to the left (while looking left) and move to the right.
        hand_x_start = 1  # in front of the robot.
        hand_x_end = 4
        hand_y_start = 0  # centered
        hand_y_end = 0
        hand_z = 0  # body height

        hand_vec3_start = geometry_pb2.Vec3(x=hand_x_start, y=hand_y_start, z=hand_z)
        hand_vec3_end = geometry_pb2.Vec3(x=hand_x_end, y=hand_y_end, z=hand_z)

        # We specify an orientation for the hand, which the robot will use its remaining degree
        # of freedom to achieve.  Most of it will be ignored in favor of the gaze direction.
        qw = 1
        qx = 0
        qy = 0
        qz = 0
        quat = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

        # Build a trajectory
        hand_pose1_in_flat_body = geometry_pb2.SE3Pose(position=hand_vec3_start, rotation=quat)
        hand_pose2_in_flat_body = geometry_pb2.SE3Pose(position=hand_vec3_end, rotation=quat)

        hand_pose1_in_vision = vision_T_flat_body * math_helpers.SE3Pose.from_obj(
            hand_pose1_in_flat_body)
        hand_pose2_in_vision = vision_T_flat_body * math_helpers.SE3Pose.from_obj(
            hand_pose2_in_flat_body)

        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(pose=hand_pose1_in_vision.to_proto())

        # We'll make this trajectory the same length as the one above.
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=hand_pose2_in_vision.to_proto(),
            time_since_reference=duration_pb2.Duration(seconds=duration_seconds,
                                                       nanos=duration_nanos))

        hand_traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2])

        # Build the proto
        gaze_cmd = arm_command_pb2.GazeCommand.Request(target_trajectory_in_frame1=gaze_traj,
                                                       frame1_name=VISION_FRAME_NAME,
                                                       tool_trajectory_in_frame2=hand_traj,
                                                       frame2_name=VISION_FRAME_NAME)
        arm_command = arm_command_pb2.ArmCommand.Request(arm_gaze_command=gaze_cmd)
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
        gaze_arm_command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

        # Tell the robot's body to follow the arm

        offset = geometry_pb2.Vec3(x=1, y=0, z=0)

        mobility_command = mobility_command_pb2.MobilityCommand.Request(
            follow_arm_request=basic_command_pb2.FollowArmCommand.Request(body_offset_from_hand=offset))
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            mobility_command=mobility_command)
        follow_arm_command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

        # # Combine the arm and mobility commands into one synchronized command.
        synchro_command = RobotCommandBuilder.build_synchro_command(follow_arm_command, gaze_arm_command)

        # Send the request
        gaze_command_id = command_client.robot_command(synchro_command)
        robot.logger.info('Sending combined command.')

        # Wait until the robot completes the gaze before issuing the next command.
        block_until_arm_arrives(command_client, gaze_command_id, timeout_sec=hand_traj_time + 3.0)

        # # Move the arm to a spot in front of the robot, and command the body to follow the hand.
        # # Build a position to move the arm to (in meters, relative to the body frame origin.)
        # x = 1.25
        # y = 0
        # z = 0.25
        # hand_pos_rt_body = geometry_pb2.Vec3(x=x, y=y, z=z)

        # # Rotation as a quaternion.
        # qw = 1
        # qx = 0
        # qy = 0
        # qz = 0
        # body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

        # # Build the SE(3) pose of the desired hand position in the moving body frame.
        # body_T_hand = geometry_pb2.SE3Pose(position=hand_pos_rt_body, rotation=body_Q_hand)

        # # Transform the desired from the moving body frame to the odom frame.
        # robot_state = robot_state_client.get_robot_state()
        # odom_T_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
        #                             ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        # odom_T_hand = odom_T_body * math_helpers.SE3Pose.from_obj(body_T_hand)

        # # duration in seconds
        # seconds = 5

        # # Create the arm command.
        # arm_command = RobotCommandBuilder.arm_pose_command(
        #     odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
        #     odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds)

        # Tell the robot's body to follow the arm

        # #KEEP
        # follow_arm_command = RobotCommandBuilder.follow_arm_command()

        # # Combine the arm and mobility commands into one synchronized command.
        # command = RobotCommandBuilder.build_synchro_command(follow_arm_command, arm_command)

        # # Send the request
        # move_command_id = command_client.robot_command(command)
        # robot.logger.info('Moving arm to position.')

        # block_until_arm_arrives(command_client, move_command_id, 6.0)

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
        gaze_lock_body_follow(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception("Threw an exception")
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
