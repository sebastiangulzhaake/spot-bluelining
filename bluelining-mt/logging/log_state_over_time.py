# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Simple robot state capture tutorial."""

import csv
import sys
import time

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

    pos = np.zeros(3)
    rot = np.zeros(4)

    #odom = np.array(np.array())

    f = open('Robot_state_logging.csv', 'w')
    header = ['pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w']
    writer = csv.writer(f)
    writer.writerow(header)

    key = 'hand'

    samples = 0

    # Make a robot state request
    while samples < 300:  # making a loop
    

        state = robot_state_client.get_robot_state()
        transforms = state.kinematic_state.transforms_snapshot.child_to_parent_edge_map
        pos[0] = transforms.get(key).parent_tform_child.position.x
        pos[1] = transforms.get(key).parent_tform_child.position.y
        pos[2] = transforms.get(key).parent_tform_child.position.z
        rot[0] = transforms.get(key).parent_tform_child.rotation.x
        rot[1] = transforms.get(key).parent_tform_child.rotation.y
        rot[2] = transforms.get(key).parent_tform_child.rotation.z
        rot[2] = transforms.get(key).parent_tform_child.rotation.w

        data = [pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3]]
        writer.writerow(data)
        f.flush()
        #print(transforms.get('odom'))
        print("pos: ", "x: ", pos[0], "y: ", pos[1], "z: ", pos[2],
        "rot: ", "x: ", rot[0], "y: ", rot[1], "z: ", rot[2], "w: ", rot[3], end='\r')
        samples += 1
        time.sleep(1)

            


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



