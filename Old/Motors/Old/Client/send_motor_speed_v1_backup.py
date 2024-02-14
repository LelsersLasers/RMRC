import pickle
import time
import socket

HOST='192.168.0.210'
PORT=5025
#with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
clientSocket=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientSocket.connect((HOST, PORT))

while True:
    file=open('motorspeed.ps4','rb')
    try:
        StrNum=pickle.load(file)
        print("t:",StrNum)
        bStr=StrNum.encode('utf-8')
        clientSocket.sendall(bStr)
        data=clientSocket.recv(1024)
    except:
        print("")
    file.close()
    time.sleep(1)
