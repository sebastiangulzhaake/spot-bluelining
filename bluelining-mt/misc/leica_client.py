import os
import socket
import struct
import sys
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# host='192.168.78.125'
# port=65432 #same as server
# sock.connect((host,port))

# while(True):
#     print("connected")
#     msg = sock.recv(10000)

#     # N = msg[:4]
#     # time = msg[4:12]
#     # time_status = msg[-40:-32]
#     # x = msg[-32:-24]
#     # y = msg[-24:-16]
#     # z = msg[-16:-8]
#     # pos_status = msg[-8:]

#     # N = int.from_bytes(N, byteorder='little')
#     # time = int.from_bytes(time, byteorder='little')
#     # time_status = int.from_bytes(time_status, byteorder='little')
#     # x = struct.unpack('<d', x)
#     # y = struct.unpack('<d', y)
#     # z = struct.unpack('<d', z)
#     # pos_status = int.from_bytes(pos_status, byteorder='little')

#     # print(N, ' ', time, ' ', time_status, x, ' ', y, ' ', z, ' ', pos_status, end='\r')
#     # sys.stdout.write("\x1b[1A\x1b[2K")
#     print(msg)


#Alina has to target our IP, and it has to be changed to the correct one below

UDP_IP = "192.168.78.31" #my IP at maxiv_visitor
UDP_PORT = 65431 #whatever port Alina gives us
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))
print("ready")
while True:
    data = sock.recv(1024)
    print(data)