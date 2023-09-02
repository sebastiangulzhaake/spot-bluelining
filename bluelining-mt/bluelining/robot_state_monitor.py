import threading 

class RobotStateMonitor():
    def __init__(self):
        self._robot_state = None
        self._end_effector_force = None
        self._lock = threading.Lock()

    # Sets new robot state variables to the monitor attributes.
    # Called by robot state thread.
    def set_robot_state(self, robot_state):
        self._lock.acquire()
        self._robot_state = robot_state
        self._end_effector_force = robot_state.manipulator_state.estimated_end_effector_force_in_hand
        self._lock.release()

    # Returns the end effector force in z axis.
    # Called by controller thread.
    def get_end_effector_z_force(self):
        self._lock.acquire()
        z_force = self._end_effector_force.z
        self._lock.release()
        return z_force

    def get_robot_state(self):
        return self._robot_state