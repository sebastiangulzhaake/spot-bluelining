import select
import threading 
import struct
import socket
import numpy as np
import time

from scipy.signal import savgol_filter

class DataStreamThread(threading.Thread):
    def __init__(self, data_stream_monitor, host, port, tracker):
        threading.Thread.__init__(self, daemon=True)
        print("DataStreamThread initialized")
        self._data_stream_monitor = data_stream_monitor
        self._coords = np.zeros(2, dtype=np.float64)
        self._filtered_coords = np.array([np.nan, np.nan], dtype=np.float64)
        self._x_values = np.zeros(10, dtype=np.float64)
        self._z_values = np.zeros(10, dtype=np.float64) 
        self._sock = socket.socket()
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._host = host #ip of krypton Computer (server)
        self._port = port #same as server
        self._tracker = tracker.lower()
        self._stop_event = threading.Event()

    # Thread stop request method called from main pogram.
    def stop(self):
        self._stop_event.set()
        print("DataStreamThread stop requested. ")

    # Checks if thread is stop requested. 
    # Returns True if thread is requested to stop, False otherwise.
    def stopped(self):
        return self._stop_event.is_set()

    # Savgol filter.
    # def filter(self, x: float ,z: float):
    #     self._x_values = np.roll(self._x_values, 1)
    #     self._z_values = np.roll(self._z_values, 1)
    #     self._x_values[0] = x
    #     self._z_values[0] = z
    #     savgol_x = savgol_filter(self._x_values,9,3)
    #     savgol_z = savgol_filter(self._z_values,9,3)
    #     avg_x = np.sum(savgol_x)*0.1
    #     avg_z = np.sum(savgol_z)*0.1
    #     self._data_stream_monitor.set_filtered_coords(avg_x, avg_z)

    # Rolling avarage filter of 10 latest valuese
    # def filter(self, x: float ,z: float):
    #     self._x_values = np.roll(self._x_values, 1)
    #     self._z_values = np.roll(self._z_values, 1)
    #     self._x_values[0] = x
    #     self._z_values[0] = z
    #     avg_x = np.sum(self._x_values)*0.1
    #     avg_z = np.sum(self._z_values)*0.1
    #     self._data_stream_monitor.set_filtered_coords(avg_x, avg_z)

    #No filtering
    def filter(self, x: float ,z: float):
        self._data_stream_monitor.set_filtered_coords(x, z)

    # Data stream method for using the Leica Tracker AT401 & 403. 
    # The tansmitting computer, i.e. Alina in our case, has to target the running compututer's IP within the network. 
    # IP = my IP at maxiv_visitor wifi
    # PORT = whatever port Alina gives us.
    # IP and PORT is set in main program. 
    def leica(self):
        sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
        sock.bind((self._host, self._port))
        while True:
            # Check if thread is requested to stop.
            # If thread is requested to stop, break loop to stop thread. 
            if(self.stopped()):
                break

            sock.setblocking(0)

            ready = select.select([sock], [], [], 0.5)
            if ready[0]:
                data = sock.recv(1024)
                # Data is sent as compact scientific notation data stream in byte format. Only works for this data stream setup.
                # Example: b'X,1.8602085278209543e+003,Y,-6.2550293235395122e+002,Z,-3.0650903041556268e+002\x00'
                positionsSplit = data.split(b',')

                X = float(positionsSplit[1])
                Y = float(positionsSplit[3])
                Z = float(positionsSplit[5].decode().replace('\x00', ''))
            else:
                X = np.nan
                Y = np.nan
                Z = np.nan
            


            #data = sock.recv(1024)


            #print('x: ', X, ';      y: ', Y, ';      z: ', Z)

            self._coords[0] = X
            self._coords[1] = Z

            #Filter coord and set them in monitor
            self.filter(X,Z)
            self._data_stream_monitor.set_3d_coords(X, Y, Z)    
        
    def krypton(self):
        self._sock.connect((self._host,self._port))
        while(True):
            # Check if thread is requested to stop.
            # If thread is requested to stop, break loop to stop thread. 


            if(self.stopped()):
                break

            msg = self._sock.recv(10000)

            # Krypton data being sent in one long byte stream. 
            # Extract the different parts separately.
            N = msg[:4]
            time = msg[4:12]
            time_status = msg[-40:-32]
            x = msg[-32:-24]
            y = msg[-24:-16]
            z = msg[-16:-8]
            pos_status = msg[-8:]

            # Decode data. Data arriving as LSB First, use little endian to decode. 
            N = int.from_bytes(N, byteorder='little')
            time = int.from_bytes(time, byteorder='little')
            time_status = int.from_bytes(time_status, byteorder='little')
            x = struct.unpack('<d', x)                                          #X axis point to the side, left side when looking at the nikon
            y = struct.unpack('<d', y)                                          #Y axis point upwards when looking at the nikon
            z = struct.unpack('<d', z)                                          #Z axis point into the nikon when looking at the nikon
            pos_status = int.from_bytes(pos_status, byteorder='little')
            
            # Remove characters that comes with the data.
            x = str(x)
            X = x.replace('(', '').replace(')', '').replace(',', '')
            y = str(y)
            Y = y.replace('(', '').replace(')', '').replace(',', '')
            z = str(z)
            Z = z.replace('(', '').replace(')', '').replace(',', '')

            # Cast X and Z coordinates to float.
            X = float(X)
            Y = float(Y)
            Z = float(Z)

            self._coords[0] = X
            self._coords[1] = Z

            # Filter coordinates and set them in monitor.
            self.filter(X,Z)

            # Set 3d coords in monitor
            self._data_stream_monitor.set_3d_coords(X, Y, Z)

    def run(self):
        if(self._tracker == 'leica'):
            self.leica()
        elif(self._tracker == 'krypton'):
            self.krypton()