import pickle
import time
import socket

HOST='192.168.1.6'
PORT=5025
print('Connecting to', HOST, 'on port', PORT)
#with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
clientSocket=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientSocket.settimeout(100000)
clientSocket.connect((HOST, PORT))

while True:
    with open('motorspeed.ps4','rb') as file:
        try:
            StrNum=pickle.load(file)
        except:
            StrNum="0,0,0,0,0"
    bStr=StrNum.encode('utf-8')
    try:
        clientSocket.sendall(bStr)
        data=clientSocket.recv(1024)
        print("t:",StrNum)
    except socket.timeout:
        print("Did not receive response packet.")
        clientSocket.connect((HOST, PORT))
    #time.sleep(1)

