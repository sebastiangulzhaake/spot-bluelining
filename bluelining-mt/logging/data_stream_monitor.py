import os
import socket
import struct
import time as t
import numpy as np
import threading 
from scipy.signal import savgol_filter

class DataStreamMonitor():
    def __init__(self):
        self._coords = np.zeros(2, dtype=np.float64)
        self._filtered_coords = np.array([np.nan, np.nan], dtype=np.float64)
        self._x_values = np.zeros(10, dtype=np.float64)
        self._y_values = np.zeros(10, dtype=np.float64)

        self._coords_3d = np.array([np.nan, np.nan, np.nan], dtype=np.float64)


        self._x_value = None
        self._y_value = None
        self._z_value = None     

        self._lock = threading.Lock()

    # Sets new filtered X and Z coordinates to monitor attributes.
    # Called by robot stream thread.
    def set_filtered_coords(self, avg_x, avg_z):
        self._lock.acquire()
        self._filtered_coords[0] = avg_x
        self._filtered_coords[1] = avg_z
        self._lock.release()

    def set_3d_coords(self, x, y, z):
        self._coords_3d[0] = z
        self._coords_3d[1] = x
        self._coords_3d[2] = y

    # Returns the filtered X and Z coordinates.
    # Called by controller thread.
    def get_filtered_coords(self):
        self._lock.acquire()
        filtered_coords = np.around(self._filtered_coords, 3)
        self._lock.release()
        return filtered_coords

    def get_3d_coords(self):
        self._lock.acquire()
        coords_3d = self._coords_3d
        self._lock.release()
        return coords_3d


