import dynamixel_sdk

PORT = '/dev/ttyUSB0'
portHandler = dynamixel_sdk.PortHandler(PORT)
packetHandler = dynamixel_sdk.PacketHandler(2.0)
portHandler.openPort()
portHandler.setBaudRate(57600)

res, err = packetHandler.write1ByteTxRx(portHandler, 2, 11, 1)
if res != 0: print('Error writing Velocity Mode to ID 2.')
res, err = packetHandler.write1ByteTxRx(portHandler, 3, 11, 1)
if res != 0: print('Error writing Velocity Mode to ID 3.')

res, err = packetHandler.write1ByteTxRx(portHandler, 2, 64, 1)
if res != 0: print('Error writing Torque On to ID 2.')
res, err = packetHandler.write1ByteTxRx(portHandler, 3, 64, 1)
if res != 0: print('Error writing Torque On to ID 3.')

res, err = packetHandler.write4ByteTxRx(portHandler, 2, 104, 42)
if res != 0: print('Error setting Goal Velocity on ID 2.')
res, err = packetHandler.write4ByteTxRx(portHandler, 3, 104, 42)
if res != 0: print('Error setting Goal Velocity on ID 3.')
