import time as t
import numpy as np
import threading 
from scipy.signal import savgol_filter

class InputMonitor():
    def __init__(self):
        self._input = None
        self._lock = threading.Lock()

    # Sets new filtered X and Z coordinates to monitor attributes.
    # Called by robot stream thread.
    def set_input(self, command):
        self._lock.acquire()
        self._input = command
        self._lock.release()

    # Returns the filtered X and Z coordinates.
    # Called by controller thread.
    def get_input(self):
        self._lock.acquire()
        command = self._input
        self._input = None
        self._lock.release()
        return command