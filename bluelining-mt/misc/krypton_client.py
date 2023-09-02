import os
import socket
import struct
import sys
import csv
import time as t
import threading
from this import d
import select


sock = socket.socket()

host='192.168.18.58'
port=12000 #same as server
sock.connect((host,port))

f = open('Point_gathering.csv', 'w')
header = ['X', 'Z']
writer = csv.writer(f)
writer.writerow(header)


while(True):
    msg = sock.recv(10000)

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

    print(N, ' ', time, ' ', time_status, x, ' ', y, ' ', z, ' ', pos_status, end='\r')
    sys.stdout.write("\x1b[1A\x1b[2K")
    
    x = str(x)
    X = x.replace('(', '').replace(')', '').replace(',', '')
    z = str(z)
    Z = z.replace('(', '').replace(')', '').replace(',', '')



    # while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
    #     line = sys.stdin.readline()
    #     if line:
    #         data = [X, Z]
    #         writer.writerow(data)
    #         print('Logged state')
    #     else: # an empty line means stdin has been closed
    #         print('eof')
    #         exit(0)


    command = input()
    if command == 'l':  # if key 'l' is pressed followed by enter
        data = [X, Z]
        writer.writerow(data)
        print('Logged state: X: ', X, '     . Y: ', Z)
        f.flush()

 #   t.sleep(0.5)