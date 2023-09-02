import os
import socket
import struct
import time as t
import numpy as np
import threading 
from scipy.signal import savgol_filter
# Declraing a lock

class stream_handler():

    def __init__(self, host, port):
        self._coords = np.zeros(2, dtype=np.float64)
        self._filtered_coords = np.zeros(2, dtype=np.float64)
        self._x_values = np.zeros(10, dtype=np.float64)
        self._z_values = np.zeros(10, dtype=np.float64)     
        self._sock = socket.socket()
        self._host = host #ip of krypton Computer (server)
        self._port = port #same as server
        self._lock = threading.Lock()
        print("stream_handler initialized")

    def filter(self, x: float ,z: float):
        self._x_values = np.roll(self._x_values, 1)
        self._z_values = np.roll(self._z_values, 1)
        self._x_values[0] = x
        self._z_values[0] = z
        # filtered_x = np.sum(self._x_values)*0.1
        # filtered_z = np.sum(self._z_values)*0.1
        savgol_x = savgol_filter(self._x_values,9,3)
        savgol_z = savgol_filter(self._z_values,9,3)
        avg_x = np.sum(savgol_x)*0.1
        avg_z = np.sum(savgol_z)*0.1
        self._lock.acquire()
        # self._filtered_coords[0] =filtered_x
        # self._filtered_coords[1] =filtered_z
        self._filtered_coords[0] = avg_x
        self._filtered_coords[1] = avg_z
        self._lock.release()

    def get_filtered_coords(self):
        self._lock.acquire()
        filtered_coords = np.around(self._filtered_coords, 3)
        self._lock.release()
        return filtered_coords

    def handle(self):
        self._sock.connect((self._host,self._port))
        while(True):
            msg = self._sock.recv(10000)

            N = msg[:4]
            time = msg[4:12]
            time_status = msg[-40:-32]
            x = msg[-32:-24]
            y = msg[-24:-16]
            z = msg[-16:-8]
            pos_status = msg[-8:]

            N = int.from_bytes(N, byteorder='little')
            time = int.from_bytes(time, byteorder='little')
            time_status = int.from_bytes(time_status, byteorder='little')
            x = struct.unpack('<d', x)                                          #X axis point to the side, left side when looking at the nikon
            y = struct.unpack('<d', y)                                          #Y axis point upwards when looking at the nikon
            z = struct.unpack('<d', z)                                          #Z axis point into the nikon when looking at the nikon
            pos_status = int.from_bytes(pos_status, byteorder='little')
            
            x = str(x)
            X = x.replace('(', '').replace(')', '').replace(',', '')
            z = str(z)
            Z = z.replace('(', '').replace(')', '').replace(',', '')

            X = float(X)
            Z = float(Z)

            self._coords[0] = X
            self._coords[1] = Z

            self.filter(X,Z)


# def main():
#     host='192.168.18.165' #ip of krypton Computer (server)
#     port=12000 #same as server
#     handler = stream_handler(host,port)
#     handler.handle()

# if __name__ == "__main__":
#     if not main():
#         os._exit(1)
#     os._exit(0)
