#!/usr/bin/env python

import os, socket
import sys, tty, termios
import smbus
bus = smbus.SMBus(1)
bus2 = smbus.SMBus(8)
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

HOST='192.168.1.6'
PORT=5025
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

print("Waiting for controller input")
conn, address = s.accept()
print(address[0]+":"+str(address[1]))

# flipper = 0 means up, = 1 means down
flipper = 0
while 1:
    data = conn.recv(1024)
    try:
        conn.sendall(b'Client control data received')
    except BrokenPipeError:
        print("Broken pipe, attempting repair...")
        s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))
        s.listen(1)
        conn, address = s.accept() 
    # print('Received', repr(data))
    StrNum=bytes.decode(data)
    # print(StrNum)
    cNum=StrNum.split(',')
    #print(cNum)
    try:
        arrNum=list(map(int,cNum))
    except:
        # arrNum = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        arrNum = [0, 0, 0, 0, 0]


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
        print("FLIPPER")
        if flipper == 0:
            pos = 0
            print("UP ", pos)
            bus.write_byte(37,1)
            print("a")
            bus.write_byte(37, pos)
            print("b")
            #bus2.write_byte(37,1)
            # print("c")
            #bus2.write_byte(37, pos)
            # print("d")
            flipper = 1
            print(flipper)
        else:
            pos = 100
            print("DOWN ", pos)
            bus.write_byte(37,1)
            bus.write_byte(37, pos)
            #bus2.write_byte(37,1)  
            #bus2.write_byte(37, pos)
            flipper = 0
    """
    for i in range(5, len(arrNum)):
        bus.write_byte(37, i-3)
        bus.write_byte(37, arrNum[i])
        bus2.write_byte(37, i-3)
        bus2.write_byte(37, arrNum[i])
    """
conn.close()
s.close()
drive_4x.torqueOff()
drive_4x.closePort()
