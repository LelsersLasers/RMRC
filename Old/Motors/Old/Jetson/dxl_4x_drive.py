#!/usr/bin/env python
import os, sys, tty, termios
from dynamixel_sdk import *   

class dxl_4x:
    def __init__(self):
        # Control table address
        # anything without "self." will not be used in TBS 4x read_write_v[something].py
        # just there for aesthetical purposes & as a reminder
        self.reset = 0
        self.PROTOCOL_VERSION            = 2.0
        self.ADDR_TORQUE_ENABLE          = 64
        ADDR_LED_RED                = 65
        ADDR_OPERATING_MODE         = 11
        OP_VELOCITY_CTRL_MODE       = 1
        self.ADDR_GOAL_VELOCITY          = 104
        ADDR_VELOCITY_LIMIT         = 44
        VELOCITY_LIMIT              = 265
        self.ADDR_PRESENT_VELOCITY       = 128
        LEN_LED_RED                 = 1
        ADDR_GOAL_POSITION          = 116
        LEN_GOAL_POSITION           = 4
        ADDR_PRESENT_POSITION       = 132
        LEN_PRESENT_POSITION        = 4
        DXL_MINIMUM_POSITION_VALUE  = 0
        DXL_MAXIMUM_POSITION_VALUE  = 4095
        BAUDRATE                    = 57600
        self.DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
        self.DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
        self.DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
        self.DXL4_ID                     = 4                 # Dynamixel#1 ID : 4
        self.DXL1_orientation = 1 #1 or -1
        self.DXL2_orientation = -1 #1 or -1
        self.DXL3_orientation = -1 #1 or -1
        self.DXL4_orientation = 1 #1 or -1
        self.DXL1_leftright = -1 #1 or -1
        self.DXL2_leftright = 1 #1 or -1
        self.DXL3_leftright = 1 #1 or -1
        self.DXL4_leftright = -1 #1 or -1
        self.DXL1_left = -1 #1 or -1
        self.DXL2_left = 1 #1 or -1
        self.DXL3_left = 1 #1 or -1
        self.DXL4_left = -1 #1 or -1
        self.DXL1_right = -1 #1 or -1
        self.DXL2_right = 1 #1 or -1
        self.DXL3_right = 1 #1 or -1
        self.DXL4_right = -1 #1 or -1
        DEVICENAME                  = '/dev/ttyUSB0'
        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
        
        # Initialize PortHandler and PacketHandler
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port.")
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate.")

        #self.speedMotor=0
        self.speedMotor1=0
        self.speedMotor2=0
        self.speedMotor3=0
        self.speedMotor4=0
        #self.turnLR=0
        self.speedNowMotor1=0
        self.speedNowMotor2=0
        self.speedNowMotor3=0
        self.speedNowMotor4=0

    def torqueOn(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def torqueOff(self):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def setSpeed(self,speed,turn):
        if speed >=250:
            speed=250
        if speed <=-250:
            speed=-250
        if turn==0:
            self.speedMotor1=speed
            self.speedMotor2=speed
            self.speedMotor3=speed
            self.speedMotor4=speed
        if turn<0 and turn >-300:
            self.speedMotor1=speed + turn * self.DXL1_left
            self.speedMotor2=speed + turn * self.DXL2_left
            self.speedMotor3=speed + turn * self.DXL3_left
            self.speedMotor4=speed + turn * self.DXL4_left
        if turn>0 and turn <300:
            self.speedMotor1=speed + turn * self.DXL1_right
            self.speedMotor2=speed + turn * self.DXL2_right
            self.speedMotor3=speed + turn * self.DXL3_right
            self.speedMotor4=speed + turn * self.DXL4_right
        if turn>=300:
            self.speedMotor1=speed * self.DXL1_leftright
            self.speedMotor2=speed * self.DXL2_leftright
            self.speedMotor3=speed * self.DXL3_leftright
            self.speedMotor4=speed * self.DXL4_leftright
        if turn<=-300:
            self.speedMotor1= -speed * self.DXL1_leftright
            self.speedMotor2= -speed * self.DXL2_leftright
            self.speedMotor3= -speed * self.DXL3_leftright
            self.speedMotor4= -speed * self.DXL4_leftright
        if self.speedMotor1>250:
            self.speedMotor1=250
        if self.speedMotor2>250:
            self.speedMotor2=250
        if self.speedMotor3>250:
            self.speedMotor3=250
        if self.speedMotor4>250:
            self.speedMotor4=250
        if self.speedMotor1<-250:
            self.speedMotor1=-250
        if self.speedMotor2<-250:
            self.speedMotor2=-250
        if self.speedMotor3<-250:
            self.speedMotor3=-250
        if self.speedMotor4<-250:
            self.speedMotor4=-250

    def updateSpeed(self):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_GOAL_VELOCITY, self.speedMotor1 * self.DXL1_orientation)
        if dxl_comm_result != COMM_SUCCESS:
            print("1 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("2 %s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_GOAL_VELOCITY, self.speedMotor2 * self.DXL2_orientation)
        if dxl_comm_result != COMM_SUCCESS:
            print("1 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("2 %s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_GOAL_VELOCITY, self.speedMotor3 * self.DXL3_orientation)
        if dxl_comm_result != COMM_SUCCESS:
            print("1 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("2 %s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_GOAL_VELOCITY, self.speedMotor4 * self.DXL4_orientation)
        if dxl_comm_result != COMM_SUCCESS:
            print("1 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("2 %s" % self.packetHandler.getRxPacketError(dxl_error))
        self.getCurrentSpeed()

    def getCurrentSpeed(self):
        self.speedNowMotor1, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRESENT_VELOCITY)
        self.speedNowMotor2, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRESENT_VELOCITY)
        self.speedNowMotor3, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRESENT_VELOCITY)
        self.speedNowMotor4, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRESENT_VELOCITY)
        # convert unsigned integer values to signed integer for readability
        errorCode1, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, 70)
        errorCode2, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL2_ID, 70)
        errorCode3, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL3_ID, 70)
        errorCode4, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL4_ID, 70)
        if errorCode1>0 or errorCode2>0 or errorCode3>0 or errorCode4>0:
            print(errorCode1,errorCode2,errorCode3,errorCode4)
            self.reset_motors()

        if self.speedNowMotor1>1024: 
            self.speedNowMotor1-=2**32
        if self.speedNowMotor2>1024:
            self.speedNowMotor2-=2**32
        if self.speedNowMotor3>1024:
            self.speedNowMotor3-=2**32
        if self.speedNowMotor4>1024:
            self.speedNowMotor4-=2**32

    def reset_motors(self):
        self.packetHandler.reboot(self.portHandler, self.DXL1_ID)
        self.packetHandler.reboot(self.portHandler, self.DXL2_ID)
        self.packetHandler.reboot(self.portHandler, self.DXL3_ID)
        self.packetHandler.reboot(self.portHandler, self.DXL4_ID)

    def closePort(self):
        self.portHandler.closePort()
