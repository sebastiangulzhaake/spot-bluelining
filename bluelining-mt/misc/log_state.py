# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Simple robot state capture tutorial."""

import sys

import bosdyn.client
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
import keyboard  # using module keyboard
import numpy as np

def main():
    import argparse

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args()

    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('RobotStateClient')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    vision = np.array(np.array())
    odom = np.array(np.array())

    # Make a robot state request
    while True:  # making a loop
     
        command = input()
        if command == 'l':  # if key 'l' is pressed followed by enter
            print('Logged state')
            state = robot_state_client.get_robot_state()
            transforms = state.kinematic_state.transforms_snapshot.child_to_parent_edge_map
            print(transforms.get('vision').parent_tform_child.position.x)
            print(transforms.get('vision').parent_tform_child.position.y)
            print(transforms.get('vision').parent_tform_child.position.z)
            vision[0[0]] = transforms.get('vision').parent_tform_child.position.x
            vision[0[1]] = transforms.get('vision').parent_tform_child.position.y
            vision[0[2]] = transforms.get('vision').parent_tform_child.position.z
            print(transforms.get('odom'))

            


if __name__ == "__main__":
    if not main():
        sys.exit(1)



