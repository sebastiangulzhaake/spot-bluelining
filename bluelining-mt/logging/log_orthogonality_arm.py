# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Simple robot state capture tutorial."""

import csv
import sys
import time
import numpy as np

import bosdyn.client
from bosdyn.client.frame_helpers import (GRAV_ALIGNED_BODY_FRAME_NAME, VISION_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b,
                                         get_vision_tform_body)
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.math_helpers import Quat
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, block_for_trajectory_cmd,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.world_object import WorldObjectClient
from scipy.spatial.transform import Rotation as R

from bosdyn.api import world_object_pb2
from bosdyn.api import geometry_pb2, arm_command_pb2, synchronized_command_pb2, robot_command_pb2

from data_stream_monitor import DataStreamMonitor
from data_stream_thread import DataStreamThread

def main():

    def arm_cart_velocity_cmd_helper(u, u_z, end_time, frame):

        cart_coord = geometry_pb2.Vec3(x=u[0], y=u[1], z=u_z)

        cart_velocity = arm_command_pb2.ArmVelocityCommand.CartesianVelocity(frame_name=frame, velocity_in_frame_name=cart_coord)

        angular_vel = geometry_pb2.Vec3(x=0, y=0, z=0)

        cart_velocity_command = arm_command_pb2.ArmVelocityCommand.Request(
            cartesian_velocity=cart_velocity,
            angular_velocity_of_hand_rt_odom_in_hand=angular_vel)

        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_velocity_command=cart_velocity_command)
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command)
        robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

        synchro_command = RobotCommandBuilder.build_synchro_command(robot_cmd)
        
        cmd_id = command_client.robot_command(command=synchro_command,
                                            end_time_secs=time.time() + end_time)
    
    def log_position_during_command(end_time, speed):
        start_time = time.time()
        curr_time = time.time()
        while curr_time - start_time < end_time:
            pos = data_stream_monitor.get_3d_coords()
            data = [pos[0], pos[1], pos[2], speed]
            writer.writerow(data)
            f.flush()
            time.sleep(0.1)
            curr_time = time.time()
    def move_arm_orthogonaly(speed, frame, samples=0):

        start_speed = speed

        while samples < 6:  # making a loop
            speed = start_speed - samples*0.01

            end_time = distance/speed
            #move arm forward 0.2 m
            arm_cart_velocity_cmd_helper([speed, 0], 0, end_time, frame)

            log_position_during_command(end_time, speed)

            #move arm backward 0.2 m
            arm_cart_velocity_cmd_helper([-speed, 0], 0, end_time, frame)

            log_position_during_command(end_time, speed)

            #move arm backward 0.2 m
            arm_cart_velocity_cmd_helper([-speed, 0], 0, end_time, frame)

            log_position_during_command(end_time, speed)

            #move arm forward 0.2 m
            arm_cart_velocity_cmd_helper([speed, 0], 0, end_time, frame)

            log_position_during_command(end_time, speed)

            #move arm left 0.2 m
            arm_cart_velocity_cmd_helper([0, speed], 0, end_time, frame)

            log_position_during_command(end_time, speed)

            #move arm right 0.2 m
            arm_cart_velocity_cmd_helper([0, -speed], 0, end_time, frame)

            log_position_during_command(end_time, speed)

            #move arm right 0.2 m
            arm_cart_velocity_cmd_helper([0, -speed], 0, end_time, frame)

            log_position_during_command(end_time, speed)

            #move arm left 0.2 m
            arm_cart_velocity_cmd_helper([0, speed], 0, end_time, frame)

            log_position_during_command(end_time, speed)
            
            # Unstow the arm
            unstow = RobotCommandBuilder.arm_ready_command()

            # Issue the command via the RobotCommandClient
            unstow_command_id = command_client.robot_command(unstow)
            robot.logger.info("Unstow command issued.")

            block_until_arm_arrives(command_client, unstow_command_id, 3.0)

            samples += 1

    import argparse

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args()

    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('RobotStateClient')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    pos = np.zeros(3)
    rot = np.zeros(4)

    #odom = np.array(np.array())

    distance = 0.2
    speed = 0.1
    frame = GRAV_ALIGNED_BODY_FRAME_NAME

    host='192.168.18.58' #ip of krypton Computer (server)
    port=12000 #same as server
    data_stream_monitor = DataStreamMonitor()
    tracker = 'krypton'
    #tracker = 'leica'
    data_stream_thread = DataStreamThread(data_stream_monitor, host, port, tracker)
    data_stream_thread.start()

    robot.start_time_sync()

    # Verify the robot has an arm
    assert robot.has_arm(), "Robot requires an arm to run this example."
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                        "such as the estop SDK example, to configure E-Stop."

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
        # RobotCommandBuilder for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        robot.logger.info("Commanding robot to stand...")

        blocking_stand(command_client, timeout_sec=10)

        robot.logger.info("Robot standing.")
        unstow = RobotCommandBuilder.arm_ready_command()

        # Issue the command via the RobotCommandClient
        unstow_command_id = command_client.robot_command(unstow)
        robot.logger.info("Unstow command issued.")

        block_until_arm_arrives(command_client, unstow_command_id, 3.0)

        f = open('Arm_orthogonality_logging_flat_body.csv', 'w')
        header = ['pos_x', 'pos_y', 'pos_z', 'speed']
        writer = csv.writer(f)
        writer.writerow(header)
        move_arm_orthogonaly(speed, frame=GRAV_ALIGNED_BODY_FRAME_NAME)
        f.close()

        f = open('Arm_orthogonality_logging_vision.csv', 'w')
        header = ['pos_x', 'pos_y', 'pos_z', 'speed']
        writer = csv.writer(f)
        writer.writerow(header)        
        move_arm_orthogonaly(speed, frame=VISION_FRAME_NAME)
        f.close()

        f = open('Arm_orthogonality_logging_odom.csv', 'w')
        header = ['pos_x', 'pos_y', 'pos_z', 'speed']
        writer = csv.writer(f)
        writer.writerow(header) 
        move_arm_orthogonaly(speed, frame=ODOM_FRAME_NAME)
        f.close()

if __name__ == "__main__":
    if not main():
        sys.exit(1)



"""
    child_to_parent_edge_map {
      key: "hand"
      value {
        parent_frame_name: "body"
        parent_tform_child {
          position {
            x: 0.43979611992836
            y: 0.16729873418807983
            z: 0.2559342682361603
          }
          rotation {
            x: -0.025643981993198395
            y: 0.014037746004760265
            z: 0.5324918031692505
            w: 0.8459301590919495
          }
        }
      }
    }
"""



