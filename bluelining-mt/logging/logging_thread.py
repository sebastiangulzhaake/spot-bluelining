import select
import threading 
import struct
import socket
import numpy as np
import time

from scipy.signal import savgol_filter

class DataStreamThread(threading.Thread):
    def __init__(self, data_stream_monitor, f):
        threading.Thread.__init__(self, daemon=True)
        print("LoggingThread initialized")
        self._data_stream_monitor = data_stream_monitor
        self._stop_event = threading.Event()

    # Thread stop request method called from main pogram.
    def stop(self):
        self._stop_event.set()
        print("DataStreamThread stop requested. ")

    # Checks if thread is stop requested. 
    # Returns True if thread is requested to stop, False otherwise.
    def stopped(self):
        return self._stop_event.is_set()

            # Set 3d coords in monitor
    self._data_stream_monitor.get_3d_coords(X, Y, Z)

    def run(self):
        if(self._tracker == 'leica'):
            self.leica()
        elif(self._tracker == 'krypton'):
            self.krypton()