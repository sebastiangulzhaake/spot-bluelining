import time
import threading 

class RobotStateThread(threading.Thread):
    def __init__(self, robot_state_client, robot_state_monitor):
        threading.Thread.__init__(self, daemon=True)
        print("RobotStateThread initialized")
        self._robot_state_client = robot_state_client
        self._robot_state_monitor = robot_state_monitor
        self._stop_event = threading.Event()

    # Thread stop request method called from main pogram
    def stop(self):
        self._stop_event.set()
        print("RobotStateThread stop requested. ")

    # Checks if thread is stop requested. 
    # Returns True if thread is requested to stop, False otherwise.
    def stopped(self):
        return self._stop_event.is_set()

    def run(self):    
        while(True):
            # Check if thread is requested to stop.
            # If thread is requested to stop, break loop to stop thread. 
            if(self.stopped()):
                break
            t0 = time.time()
            robot_state = self._robot_state_client.get_robot_state()
            self._robot_state_monitor.set_robot_state(robot_state)
            t1 = time.time()
            t_diff = t1-t0
            if t_diff < 0.1:
                time.sleep(0.1 - (t1 - t0))