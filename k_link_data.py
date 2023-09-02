import socket
import struct
import sys

HOST = 'localhost'
PORT = 50506

"""Open specified port and return file-like object"""
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


sock.connect((HOST, PORT))

msg = sock.recv(10000)
print("stream initialized")

network_HOST = '192.168.18.101'
network_PORT = 12000
network_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
network_sock.bind((network_HOST, network_PORT))
network_sock.listen(10)
conn, addr = network_sock.accept()
if conn:
	print("client connected")
while(True):
		
	while msg:

		try:
			#print(msg)
			N = msg[:4]
			time = msg[4:12]
			time_status = msg[-40:-32]
			x = msg[-32:-24]
			y = msg[-24:-16]
			z = msg[-16:-8]
			pos_status = msg[-8:]

			conn.send(msg)

			msg = sock.recv(10000)
		except:
			network_sock.listen(10)
			conn, addr = network_sock.accept()
			if conn:
				print("client connected")