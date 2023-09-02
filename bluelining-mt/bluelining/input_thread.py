import threading 
import time

class InputThread(threading.Thread):
    def __init__(self, input_monitor):
        threading.Thread.__init__(self, daemon=True)
        print("InputThread initialized")
        self._stop_event = threading.Event()
        self._input_monitor = input_monitor

    # Thread stop request method called from main pogram
    def stop(self):
        self._stop_event.set()
        print("InputThread stop requested. ")

    # Checks if thread is stop requested. 
    # Returns True if thread is requested to stop, False otherwise.
    def stopped(self):
        return self._stop_event.is_set()

    def run(self):    
        while(True):
            if(self.stopped()):
                break
            command = input()
            self._input_monitor.set_input(command)