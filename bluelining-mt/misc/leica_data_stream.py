import socket
UDP_IP = "192.168.78.227"
UDP_PORT = 65432
sock = socket.socket(socket.AF_INET, # Internet
                  socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    #data, addr = sock.recvfrom(1024)
    data = sock.recv(10000)
    print ("received message:", data)