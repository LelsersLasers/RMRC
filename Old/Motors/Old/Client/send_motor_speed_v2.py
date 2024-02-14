import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '192.168.0.210'
port = 5025
s.connect((host, port))
while True:
    with open('motorspeed.ps4', 'rb') as fin:
        bytes_in = fin.read(1024)
        s.send(bytes_in)
s.close()