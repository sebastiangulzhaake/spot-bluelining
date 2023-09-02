import csv
import threading 
import time
import numpy as np

from bosdyn.api import world_object_pb2
from bosdyn.api import geometry_pb2, arm_command_pb2, synchronized_command_pb2, robot_command_pb2
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b, get_se2_a_tform_b,
                                         get_vision_tform_body)
from bosdyn.client.math_helpers import Quat, SE2Pose
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, block_for_trajectory_cmd,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.world_object import WorldObjectClient
from scipy.spatial.transform import Rotation as R



class ControllerThread(threading.Thread):

    def __init__(self, robot, command_client, data_stream_monitor, robot_state_monitor, goalpoints, input_monitor):
        threading.Thread.__init__(self, daemon=True)
        print("ControlThread initialized")
        self._robot = robot
        self._command_client = command_client
        self._world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)

        self._rotation = None
        self._setpoint = None
        self._data_stream_monitor = data_stream_monitor
        self._robot_state_monitor = robot_state_monitor
        self._input_monitor = input_monitor
        self._stop_event = threading.Event()
        self._reflector_height = 70-26# +105 if using the high position and 26,4 instead of 26 for Leica
        self._offsets = np.zeros(2)
        self._offset_compensation = False

        self._fiducial_rt_world = None

        self._goalpoints = goalpoints

        self._f = open('Control_xyz_logging.csv', 'w')
        header = ['error_x', 'error_y', 'u_x', 'u_y', 'f_z', 'u_z', 'time (ms)']
        self._writer = csv.writer(self._f)
        self._writer.writerow(header)

    # Thread stop request method called from main pogram.
    def stop(self):
        self._stop_event.set()
        print("ControlThread stop requested. ")

    # Checks if thread is stop requested. 
    # Returns True if thread is requested to stop, False otherwise.
    def stopped(self):
        return self._stop_event.is_set()

    # Wait until the operator confirms tracking
    def wait_for_tracker(self):
        print("Press 'c' to continue. Make sure you have tracking before continuing")

        command = self._input_monitor.get_input()
        while command != 'c':
            command = self._input_monitor.get_input()
        return

    # Update the current tracked error
    def update_error(self):
        # Measured current position of the reflector
        reflector_pos = self._data_stream_monitor.get_filtered_coords()
        # Calculate error vector
        tracker_e = self._setpoint - reflector_pos
        # Flip error vector to match how they are denoted elsewhere
        tracker_e_flipped = np.flip(tracker_e)
        # Transpose error vector
        tracker_e_flipped_T = np.reshape(tracker_e_flipped, (2, 1))
        # Matrix multiply with Spot's rotation matrix
        spot_e_T = np.matmul(self._rotation, tracker_e_flipped_T)
        spot_e = np.array([float(spot_e_T[0][0]), float(spot_e_T[1][0])], dtype=np.float64)

        return spot_e

    # Helper function for sending a cartesian velocity command to Spot's arm
    def arm_cart_velocity_cmd_helper(self, u, u_z, end_time):
        # Generate coordinates from control signal
        cart_coord = geometry_pb2.Vec3(x=u[0], y=u[1], z=u_z)

        # Build the velocity command
        cart_velocity = arm_command_pb2.ArmVelocityCommand.CartesianVelocity(frame_name=GRAV_ALIGNED_BODY_FRAME_NAME, velocity_in_frame_name=cart_coord)

        angular_vel = geometry_pb2.Vec3(x=0, y=0, z=0)

        # Build the full command
        cart_velocity_command = arm_command_pb2.ArmVelocityCommand.Request(
            cartesian_velocity=cart_velocity,
            angular_velocity_of_hand_rt_odom_in_hand=angular_vel)

        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_velocity_command=cart_velocity_command)
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command)
        robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

        synchro_command = RobotCommandBuilder.build_synchro_command(robot_cmd)
        
        cmd_id = self._command_client.robot_command(command=synchro_command,
                                            end_time_secs=time.time() + end_time)

    # Function for freezing/locking arm in place
    def freeze_arm(self):
        arm_traj_point = arm_command_pb2.ArmJointTrajectoryPoint()
        
        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[arm_traj_point])

        arm_joint_move_cmd = arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_traj)

        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_joint_move_command=arm_joint_move_cmd)
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command)
        robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
        self._command_client.robot_command(command=robot_cmd, end_time_secs=time.time()+5)

    # Helper function for finding the angular offset of the floor plane
    def find_floor_offset_angles(self):
        x_pos = np.zeros(3)
        y_pos = np.zeros(3)
        z_pos = np.zeros(3)

        # Find current x and y coordinates of gripper relative to body from robot state
        robot_state = self._robot_state_monitor.get_robot_state()
        transforms = robot_state.kinematic_state.transforms_snapshot.child_to_parent_edge_map
        x_start = transforms.get('hand').parent_tform_child.position.x
        y_start = transforms.get('hand').parent_tform_child.position.y
        z_start = transforms.get('hand').parent_tform_child.position.z

        # Move gripper 0.75 m in positive x
        default_command = RobotCommandBuilder.arm_pose_command(x=x_start+0.075, y=y_start, z=z_start, qw=1, qx=0, qy=0, qz=0, frame_name=GRAV_ALIGNED_BODY_FRAME_NAME, seconds=1)
        default_cmd_id = self._command_client.robot_command(default_command)
        block_until_arm_arrives(self._command_client, default_cmd_id, 3.0)

        # Press gripper into floor 
        u = [0, 0]
        self.arm_cart_velocity_cmd_helper(u, -0.01, 2)
        
        # Save x, y, z coordinates (1)
        time.sleep(2)

        start_time = time.time()
        curr_time = time.time()
        while curr_time - start_time < 2:
            self.freeze_arm()
            curr_time = time.time()

        pos = self._data_stream_monitor.get_3d_coords()
        x_pos[0] = pos[0]
        y_pos[0] = pos[1]
        z_pos[0] = pos[2]

        # Move gripper x: -75 mm & y: -75 mm
        default_command = RobotCommandBuilder.arm_pose_command(x=x_start-0.075, y=y_start-0.075, z=z_start, qw=1, qx=0, qy=0, qz=0, frame_name=GRAV_ALIGNED_BODY_FRAME_NAME, seconds=1)
        default_cmd_id = self._command_client.robot_command(default_command)
        block_until_arm_arrives(self._command_client, default_cmd_id, 3.0)

        # Press gripper into floor 
        u = [0, 0]
        self.arm_cart_velocity_cmd_helper(u, -0.01, 2)

        # Save x, y, z coordinates (2)
        time.sleep(2)

        start_time = time.time()
        curr_time = time.time()
        while curr_time - start_time < 2:
            self.freeze_arm()
            curr_time = time.time()

        pos = self._data_stream_monitor.get_3d_coords()
        x_pos[1] = pos[0]
        y_pos[1] = pos[1]
        z_pos[1] = pos[2]

        # Move gripper x: -75 mm & y: 75 mm
        default_command = RobotCommandBuilder.arm_pose_command(x=x_start-0.075, y=y_start+0.075, z=z_start, qw=1, qx=0, qy=0, qz=0, frame_name=GRAV_ALIGNED_BODY_FRAME_NAME, seconds=1)
        default_cmd_id = self._command_client.robot_command(default_command)
        block_until_arm_arrives(self._command_client, default_cmd_id, 3.0)

        # Press gripper into floor 
        u = [0, 0]
        self.arm_cart_velocity_cmd_helper(u, -0.01, 2)        # Press gripper into floor 

        # Save x, y, z coordinates (3)
        time.sleep(2)

        start_time = time.time()
        curr_time = time.time()
        while curr_time - start_time < 2:
            self.freeze_arm()
            curr_time = time.time()

        pos = self._data_stream_monitor.get_3d_coords()
        x_pos[2] = pos[0]
        y_pos[2] = pos[1]
        z_pos[2] = pos[2]
        print('x_pos: ', x_pos)
        print('y_pos: ', y_pos)
        print('z_pos: ', z_pos)

        # Take smallest and biggest x coordinate and their respective Z to calculate angle in x
        x_index_max = np.argmax(x_pos)
        x_index_min = np.argmin(x_pos)
        x_max = x_pos[x_index_max]
        z_for_x_max = z_pos[x_index_max]
        x_min = x_pos[x_index_min]
        z_for_x_min = z_pos[x_index_min]

        # Calculate the offset in x
        z_diff = z_for_x_max - z_for_x_min
        print('z_diff in x: ', z_diff)
        x_diff = x_max - x_min
        print('x_diff in x: ', x_diff)
        alpha = np.arctan(z_diff/x_diff)
        print('alpha in x: ', alpha)
        l = np.tan(alpha) * self._reflector_height
        x_offset = np.cos(alpha) * l

        # Take smallest and biggest x coordinate and their respective Z to calculate angle in y
        y_index_max = np.argmax(y_pos)
        y_index_min = np.argmin(y_pos)
        y_max = y_pos[y_index_max]
        z_for_y_max = z_pos[y_index_max]
        y_min = y_pos[y_index_min]
        z_for_y_min = z_pos[y_index_min]

        # Calculate the offset in y
        z_diff = z_for_y_max - z_for_y_min
        print('z_diff in y: ', z_diff)
        y_diff = y_max - y_min
        print('y_diff in y: ', y_diff)
        alpha = np.arctan(z_diff/y_diff)
        print('alpha in y: ', alpha)
        l = np.tan(alpha) * self._reflector_height
        y_offset = np.cos(alpha) * l

        # Return x and y offset
        self._offsets = np.array([x_offset, y_offset])
        return

    # Function for finding the angular offset of the floor plane, special variant of function "arm_controller"
    def offset_compensation(self):
        e = np.zeros(2, dtype=np.float64)
        Kp = 0.0005
        Ki = 0.0001
        e_prev = np.array([0, 0])
        t_prev = time.time()*1000
        I = np.zeros(2, dtype=np.float64)
        I_max = np.array([0.001, 0.001], dtype=np.float64)
        u_max = np.array([0.3, 0.3], dtype=np.float64)

        #Value in hz
        control_freq = 5.0
        loop_duration = 1.0/control_freq
        setpoint_z = 2.0
        Kp_z = 0.01

        u_z = 0.0

        end_time = 2

        stopflag = False
        lastflag = False

        while True:
            # Check if thread is requested to stop from main:
            # If thread is requested to stop, close log file and break the loop
            if(self.stopped()):
                self._f.close()
                return

            # Update error and time
            t = time.time()*1000
            e = self.update_error()
            if np.isnan(e[0]) or np.isnan(e[1]):
                e = e_prev
                data = [0, 0, 0, 0, 0, 0, 0]
                self._writer.writerow(data)
                self._f.flush()
            
            print("error: ", e, end='\r')
            force_z = self._robot_state_monitor.get_end_effector_z_force()

            if(np.sqrt(e[0]**2 + e[1]**2) < 10 and force_z > 0):
                self.find_floor_offset_angles()
                return

            
            P = Kp*e
            I = I + Ki*e*(t - t_prev)

            # Windupcheck
            if I[0] > I_max[0]:
                I[0] = I_max[0]
            elif I[0] < -I_max[0]:
                I[0] = -I_max[0]
            if I[1] > I_max[1]:
                I[1] = I_max[1]
            elif I[1] < -I_max[1]:
                I[1] = -I_max[1]
            
            u = P + I

            if u[0] > u_max[0]:
                u[0] = u_max[0]
            elif u[0] < -u_max[0]:
                u[0] = -u_max[0]
            if u[1] > u_max[1]:
                u[1] = u_max[1]
            elif u[1] < -u_max[1]:
                u[1] = -u_max[1]

            e_z = setpoint_z - force_z
            u_z = -e_z*Kp_z

            if u_z < -0.1:
                u_z = -0.1
            elif u_z > 0.0:
                u_z = 0.0   

            if lastflag == False:
                if not np.isnan(u[0]) or not np.isnan(u[1]):
                    self.arm_cart_velocity_cmd_helper(u, u_z, end_time)

            # Update stored data for next iteration
            t_prev = t
            e_prev = e

            t1 = time.time()

            data = [e[0], e[1], u[0], u[1], force_z, u_z, t1-t]
            self._writer.writerow(data)
            self._f.flush()
            if t1 - (t*0.001) < loop_duration:
                time.sleep(loop_duration - (t1-(t*0.001)))

    # Function for controlling Spot's arm
    def arm_controller(self):
        e = np.zeros(2, dtype=np.float64)
        Kp = 0.0005
        Ki = 0.0001
        e_prev = np.array([0, 0])
        t_prev = time.time()*1000
        I = np.zeros(2, dtype=np.float64)
        I_max = np.array([0.001, 0.001], dtype=np.float64)
        u_max = np.array([0.3, 0.3], dtype=np.float64)

        # Value in hz
        control_freq = 5.0
        loop_duration = 1.0/control_freq
        setpoint_z = 2.0
        Kp_z = 0.01

        u_z = 0.0

        end_time = 2

        stopflag = False
        lastflag = False

        while True:
            # Check if thread is requested to stop from main:
            # If thread is requested to stop, close log file and break the loop
            if(self.stopped()):
                self._f.close()
                return

            # Update error and time
            t = time.time()*1000
            e = self.update_error()
            if np.isnan(e[0]) or np.isnan(e[1]):
                e = e_prev
                data = [0, 0, 0, 0, 0, 0, 0]
                self._writer.writerow(data)
                self._f.flush()
            
            print("error: ", e, end='\r')

            force_z = self._robot_state_monitor.get_end_effector_z_force()

            if(np.sqrt(e[0]**2 + e[1]**2) < 0.5 and force_z > 0):
                start_time = time.time()
                curr_time = time.time()
                while curr_time - start_time < 2:
                    self.freeze_arm()
                    curr_time = time.time()
                    lastflag = True
                if not stopflag:
                    print("Stopped!")
                    stopflag = True

            command = self._input_monitor.get_input()
            if command == 'b':
                return
            if command == 'c':
                lastflag = False
                stopflag = False

            P = Kp*e
            I = I + Ki*e*(t - t_prev)

            # Windupcheck
            if I[0] > I_max[0]:
                I[0] = I_max[0]
            elif I[0] < -I_max[0]:
                I[0] = -I_max[0]
            if I[1] > I_max[1]:
                I[1] = I_max[1]
            elif I[1] < -I_max[1]:
                I[1] = -I_max[1]
            
            u = P + I

            if u[0] > u_max[0]:
                u[0] = u_max[0]
            elif u[0] < -u_max[0]:
                u[0] = -u_max[0]
            if u[1] > u_max[1]:
                u[1] = u_max[1]
            elif u[1] < -u_max[1]:
                u[1] = -u_max[1]

            e_z = setpoint_z - force_z
            u_z = -e_z*Kp_z

            if u_z < -0.1:
                u_z = -0.1
            elif u_z > 0.0:
                u_z = 0.0   

            if lastflag == False:
                if not np.isnan(u[0]) or not np.isnan(u[1]):
                    self.arm_cart_velocity_cmd_helper(u, u_z, end_time)

            # Update stored data for next iteration
            t_prev = t
            e_prev = e

            t1 = time.time()

            data = [e[0], e[1], u[0], u[1], force_z, u_z, t1-t]
            self._writer.writerow(data)
            self._f.flush()
            if t1 - (t*0.001) < loop_duration:
                time.sleep(loop_duration - (t1-(t*0.001)))

    def power_off(self):
        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        self._robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not self._robot.is_powered_on(), "Robot power off failed."
        self._robot.logger.info("Robot safely powered off.")
        return

    # Function for finding a fiducial
    def find_fiducial(self):
        # Finds the fiducial placed on the tracker
        # Returns the fiducial frame in relation to vision frame
        # Has to be within 2-3 meters from the tracker to be able to find the fiducial
        print("Searching for fiducial")
        attempts = 0
        max_attempts = 100000

        while attempts <= max_attempts:
            # Get the first fiducial object Spot detects with the world object service.
            """Get all fiducials that Spot detects with its perception system."""
            # Get all fiducial objects (an object of a specific type).
            # Set fiducial to None if no fiducial was found.
            request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
            fiducial_objects = self._world_object_client.list_world_objects(
                object_type=request_fiducials).world_objects
            if len(fiducial_objects) > 0:
                fiducial = fiducial_objects[0]
            else:
                fiducial = None

            if fiducial is not None:
                vision_tform_fiducial = get_a_tform_b(
                    fiducial.transforms_snapshot, VISION_FRAME_NAME,
                    fiducial.apriltag_properties.frame_name_fiducial).to_proto()
                if vision_tform_fiducial is not None:
                    print("Fiducial found. ")
                    self._fiducial_rt_world = vision_tform_fiducial.position
                    break
            
            attempts += 1

    # Function for turning Spot so it faces the fiducial placed on the tracker                
    def turn_body_to_tag(self):
        print("Turn body to fiducial. ")
        # Compute the desired heading
        robot_state = self._robot_state_monitor.get_robot_state()
        robot_rt_world = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)
        robot_to_object_ewrt_world = np.array(
            [self._fiducial_rt_world.x - robot_rt_world.x, self._fiducial_rt_world.y - robot_rt_world.y, 0])
        robot_to_object_ewrt_world_norm = robot_to_object_ewrt_world / np.linalg.norm(
            robot_to_object_ewrt_world)
        xhat = robot_to_object_ewrt_world_norm
        zhat = [0.0, 0.0, 1.0]
        yhat = np.cross(zhat, xhat)
        mat = np.array([xhat, yhat, zhat]).transpose()
        angle = Quat.from_matrix(mat).to_yaw()

        robot_state = self._robot_state_monitor.get_robot_state()
        vision_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         VISION_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        # Command the robot to turn to the tag in kinematic odometry frame
        tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=vision_T_flat_body.x, goal_y=vision_T_flat_body.y,
            goal_heading=angle, frame_name=VISION_FRAME_NAME)
        
        end_time = 5.0
        # Issue the command to the robot
        cmd_id = self._command_client.robot_command(lease=None, command=tag_cmd,
                                                end_time_secs=time.time() + end_time)
        block_for_trajectory_cmd(self._command_client, cmd_id, timeout_sec=5.0)                                           

    def move_arm_to_default_pos(self, seconds=2):
        # Moving the arm to default pos
        default_command = RobotCommandBuilder.arm_pose_command(x=0.85, y=0, z=0, qw=1, qx=0, qy=0, qz=0, frame_name=GRAV_ALIGNED_BODY_FRAME_NAME, seconds=seconds)
        default_cmd_id = self._command_client.robot_command(default_command)
        # Wait for arm to reach position
        block_until_arm_arrives(self._command_client, default_cmd_id, 3.0)
        return

    def move_arm_to_extended_pos(self, seconds=2):
        # Moving the arm to default pos
        extended_command = RobotCommandBuilder.arm_pose_command(x=1, y=0, z=0, qw=1, qx=0, qy=0, qz=0, frame_name=GRAV_ALIGNED_BODY_FRAME_NAME, seconds=seconds)
        extended_cmd_id = self._command_client.robot_command(extended_command)
        # Wait for arm to reach position
        block_until_arm_arrives(self._command_client, extended_cmd_id, 3.0)
        return

    def find_robot_rotation(self):
        # Wait for robot to stabilize
        time.sleep(2)

        # Try to sample a first measurement, if no measurement is available, try again indefinitley
        pos_1 = self._data_stream_monitor.get_filtered_coords()
        while np.isnan(pos_1[0]) or np.isnan(pos_1[1]):
            pos_1 = self._data_stream_monitor.get_filtered_coords()
            if(self.stopped()):
                self._f.close()
                return

        # Unstow the arm
        self.move_arm_to_extended_pos(seconds=1)

        # Wait for robot to stabilize
        time.sleep(2)

        # Try to sample again until measurement is available
        pos_2 = self._data_stream_monitor.get_filtered_coords()
        while np.isnan(pos_2[0]) or np.isnan(pos_2[1]):
            pos_2 = self._data_stream_monitor.get_filtered_coords()

        # Calculate vector between the two positions
        v = pos_2 - pos_1
        v_len = np.sqrt(v[0]**2+v[1]**2)

        if v_len < 1:
            print("Length of vector is: ", v_len)
            self.power_off()

        # Vector in tracker's coordinate system in x/y/z with placeholder value for z
        a = [v[1], v[0], 0]

        # Vector in Spot's coordinate system in x/y/z where x is in the longditudinal direction,
        # y is transverse and z is a placeholder value
        # length of x is the same as the length of the vector in the tracker's coordinate system
        b = [v_len, 0, 0]

        # Rotation between a and b, returns a tuple where the first tuple object is a Rotation object giving us a 3x3 rotation matrix
        r = R.align_vectors(np.reshape(b,[1,3]), np.reshape(a,[1,3]))

        # Convert the Rotation object to a matrix and create 2x2 rotation matrix from the wanted elements
        rot_full = (r[0].as_matrix())
        rot = np.array([[rot_full[0][0], rot_full[0][1]], [rot_full[1][0], rot_full[1][1]]])

        return rot

    # Function for translating a point from world frame to Spot's frame
    def calculate_spot_goalpoint_from_point(self, point):
        # Wait for reflector to stop moving
        time.sleep(1)

        # Sample current reflector position
        reflector_pos = self._data_stream_monitor.get_filtered_coords()


        while np.isnan(reflector_pos[0]) or np.isnan(reflector_pos[1]):
            reflector_pos = self._data_stream_monitor.get_filtered_coords()

        # Calculate vector in tracker's frame
        vector_tracker = point - reflector_pos

        # Flip vector
        flipped_vector_tracker = np.flip(vector_tracker)

        # Transpose vector
        flipped_vector_tracker_T = np.reshape(flipped_vector_tracker, (2, 1))

        # Transform vector into Spot's coordinates in body frame
        spot_goalpoint_T = np.matmul(self._rotation, flipped_vector_tracker_T)
        spot_goalpoint = np.array([float(spot_goalpoint_T[0][0]), float(spot_goalpoint_T[1][0])], dtype=np.float64)

        return spot_goalpoint

    # Special function for backing up Spot a specified distance after locating the fiducial
    def go_to_point_in_body_stowed_arm(self, x, y, heading):
        # Get robot state and issue trajectory command in body frame that backs Spot up the specified meters
        robot_state = self._robot_state_monitor.get_robot_state()
        robot_command = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
            goal_x_rt_body=x, goal_y_rt_body=y, goal_heading_rt_body=heading,
            frame_tree_snapshot=robot_state.kinematic_state.transforms_snapshot,
            body_height=0.0)
        
        cmd_id = self._command_client.robot_command(
            command=robot_command,
            end_time_secs=time.time() + 5)
        return cmd_id

    # Helper function for commanding Spot to move to a goal point expressed in the robot's frame
    def go_to_point_in_body(self, x, y, heading):
        # Unstow the arm
        unstow = RobotCommandBuilder.arm_ready_command()

        # Issue the command via the RobotCommandClient
        unstow_command_id = self._command_client.robot_command(unstow)
        self._robot.logger.info("Unstow command issued.")
        block_until_arm_arrives(self._command_client, unstow_command_id, 3.0)

        robot_state = self._robot_state_monitor.get_robot_state()
 
        goto_rt_body = SE2Pose(x, y, heading)

        # Get an SE2 pose for odom_tform_body to convert the body-based command to a non-moving frame
        # that can be issued to the robot.
        vision_tform_body = get_se2_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                            VISION_FRAME_NAME, BODY_FRAME_NAME)
        vision_tform_goto = vision_tform_body * goto_rt_body
        robot_command = RobotCommandBuilder.synchro_se2_trajectory_command(vision_tform_goto.to_proto(),
                                                                  VISION_FRAME_NAME,
                                                                  body_height=0.0)
        # Combine with a gaze command to kepp the gripper and reflector pointed towards the tracker
        gaze_command = RobotCommandBuilder.arm_gaze_command(x=self._fiducial_rt_world.x,
                                                             y=self._fiducial_rt_world.y,
                                                             z=self._fiducial_rt_world.z+0.2,
                                                             frame_name=VISION_FRAME_NAME)
        
        synchro_command = RobotCommandBuilder.build_synchro_command(robot_command, gaze_command)
        
        cmd_id = self._command_client.robot_command(
            command=synchro_command,
            end_time_secs=time.time() + 20.0)
        return cmd_id

    # Function for commanding Spot to move to a goal point expressed in the world frame
    def go_to_point_in_tracker(self, point):
        # Takes a point expressed in the tracker's frame and move Spot to that point

        # Return to default position
        self.move_arm_to_default_pos(seconds=1)

        # Convert the point from the tracker frame to Spot's frame and convert to meters
        coords_in_spot = self.calculate_spot_goalpoint_from_point(point)
        coords_in_spot_in_meters = coords_in_spot*0.001

        # Issue command to go to point in body
        cmd_id = self.go_to_point_in_body(x=coords_in_spot_in_meters[0], y=coords_in_spot_in_meters[1], heading=0.0)

        # Print the points in Spot's body frame
        print("Points in Spot's body frame: x: ", coords_in_spot[0], "y: ", coords_in_spot[1])
        return cmd_id

    # Function for checking if we are within the envelope constrained by Spot's arm's reach
    def check_reach_envelope(self, goalpoint):

        self.move_arm_to_default_pos(seconds=1)

        reflector_pos = self._data_stream_monitor.get_filtered_coords()
        print("Reflector position: ", reflector_pos) 

        # Implementation with circle with 150 mm radius
        radius = 150
        if ((reflector_pos[0] - goalpoint[0])**2 + (reflector_pos[1] - goalpoint[1])**2 < radius**2):
            return True
        else:
            return False

    def run(self):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        self._robot.logger.info("Powering on robot... This may take a several seconds.")
        self._robot.power_on(timeout_sec=20)
        assert self._robot.is_powered_on(), "Robot power on failed."
        self._robot.logger.info("Robot powered on.")

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # RobotCommandBuilder for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        self._robot.logger.info("Commanding robot to stand...")

        blocking_stand(self._command_client, timeout_sec=10)

        self._robot.logger.info("Robot standing.")

        # Closing the gripper
        claw_gripper_close_command = RobotCommandBuilder.claw_gripper_close_command()
        close_cmd_id = self._command_client.robot_command(claw_gripper_close_command)

        # Wait for arm to close gripper fully
        block_until_arm_arrives(self._command_client, close_cmd_id, 3.0)

        # Find the fiducial on the tracker. Fiducial frame relative to vision is returned.
        self.find_fiducial()

        # Turn the body towards the tag
        self.turn_body_to_tag()

        # Back up 2 meters and wait until it is done, maybe back up depending on 
        # distance to fiducial instead, so that we always end up a fixed distance from the tracker
        # for repeatability
        cmd_id = self.go_to_point_in_body_stowed_arm(x=-1.5, y=0.0, heading=0.0)
        block_for_trajectory_cmd(self._command_client, cmd_id, timeout_sec=10.0)

        self.move_arm_to_default_pos(seconds=3)

        self.wait_for_tracker()

        # Find rotation
        self._rotation = self.find_robot_rotation()

        self.move_arm_to_default_pos(seconds=1)

        # Get the next goalpoint as a coordinate in Tracker's Y and Z
        for goalpoint in self._goalpoints:

            self._setpoint = goalpoint

            print("Goalpoint 1: ", goalpoint)

            # Check if within envelope
            if not self.check_reach_envelope(goalpoint):
                # Find rotation
                cmd_id = self.go_to_point_in_tracker(goalpoint)
                block_for_trajectory_cmd(self._command_client, cmd_id, timeout_sec=5.0) 

                # Turn the body towards the tag
                self.turn_body_to_tag()

            self.wait_for_tracker()

            # Check if within envelope, otherwise repeat until we have arrived
            while not self.check_reach_envelope(goalpoint):
                if(self.stopped()):
                    self._f.close()
                    return
                # Find rotation
                self._rotation = self.find_robot_rotation()
                cmd_id = self.go_to_point_in_tracker(goalpoint)
                block_for_trajectory_cmd(self._command_client, cmd_id, timeout_sec=5.0) 

                # Turn the body towards the tag
                self.turn_body_to_tag()

            print("Arrived within enveloppe")

            # Wait until gripper stabilizes
            time.sleep(2)
            reflector_pos = self._data_stream_monitor.get_filtered_coords()

            # Lower body to a height that accomodates a larger reaching envelope for the arm
            # Wait until body is lowered
            params = RobotCommandBuilder.mobility_params(body_height=-0.2)
            blocking_stand(self._command_client, timeout_sec=10,
                        params=params)

            self._robot.logger.info("Robot crouching.")
            
            self._rotation = self.find_robot_rotation()

            pos = self._data_stream_monitor.get_filtered_coords()

            if self._offset_compensation:
                self.offset_compensation()

                print("Offset is: ", self._offsets)

                flipped_offsets = np.flip(self._offsets)

                self._setpoint = self._setpoint - flipped_offsets
                
                self.arm_controller()
            else:
                self.arm_controller()

            self.move_arm_to_default_pos(seconds=1)

            if(self.stopped()):
                self._f.close()
                return


            