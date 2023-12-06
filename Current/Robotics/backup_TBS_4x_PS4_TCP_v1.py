#!/usr/bin/env python

import os, socket
import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

from dxl_4x_drive import *

drive_4x=dxl_4x()
drive_4x.torqueOn()

speedMotor=0
turnLeftRight=0

HOST=''
PORT=5025
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

print("Waiting for controller input")
conn, address = s.accept()
print(address[0]+":"+str(address[1]))
while 1:
    #print("1: decrease speed, 2: stop, 3: increase speed, J: left turn, K: straight, L: right turn (or press ESC to quit!)")
    data = conn.recv(1024)
    conn.sendall(b'Client control data received')
    # print('Received', repr(data))
    StrNum=bytes.decode(data)
    # print(StrNum)
    cNum=StrNum.split(',')
    #print(cNum)
    try:
        arrNum=list(map(int,cNum))
    except:
        arrNum = [0, 0, 0, 0, 0]
    #print(arrNum)

    # Set speed
    #drive_4x.setSpeed(speedMotor,turnLeftRight)
    drive_4x.speedMotor1 = arrNum[0]
    drive_4x.speedMotor2 = arrNum[1]
    drive_4x.speedMotor3 = arrNum[2]
    drive_4x.speedMotor4 = arrNum[3]
    drive_4x.reset = arrNum[4]
    drive_4x.updateSpeed()

    drive_4x.getCurrentSpeed()
    #print("Status: %d/%d %d/%d %d/%d %d/%d" % (drive_4x.speedNowMotor1, drive_4x.speedMotor1, drive_4x.speedNowMotor2, drive_4x.speedMotor2, drive_4x.speedNowMotor3, drive_4x.speedMotor3, drive_4x.speedNowMotor4, drive_4x.speedMotor4) )
    print("Status: /%d /%d /%d /%d /%d" % (drive_4x.speedMotor1, drive_4x.speedMotor2, drive_4x.speedMotor3, drive_4x.speedMotor4, drive_4x.reset) )
    if drive_4x.reset == 1:
        drive_4x.reset_motors()

conn.close()
s.close()
drive_4x.torqueOff()
drive_4x.closePort()
