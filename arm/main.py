"""
Sample Code by Dr. J for operating Dynamixel X430 motors (XL, XC, XM)

You will need to modify the name of the USB port to match your device.
This appears on line 27.

This code assumes (check these in the Wizard):
    1. You have 4 motors 
    2. With ID numbers 1-4 
    3. Set at a baud rate of 57600.  
If you have anything different, these are things to check/modify:
    1. The motor ID numbers, lines 19-22
    2. The motors list, line 24
    3. The baud rate, line 33
"""
import time
from dynamixel_sdk import *

# Make these match the actual ID numbers.  
MOTOR1_1 = 1
MOTOR1_2 = 2
MOTOR1_3 = 3
MOTOR2_1 = 4
MOTOR2_2 = 5
MOTOR2_3 = 6

# If you have more/fewer than 4 motors make sure to adjust this list
motors = [MOTOR1_1, MOTOR1_2, MOTOR1_3, MOTOR2_1, MOTOR2_2, MOTOR2_3]

# This identifies the USB port where the motor controller is attached
port = PortHandler('/dev/tty.usbserial-FT4THVLF')
# This object contains the methods for reading/writing
packet_handler = PacketHandler(2.0)

# Start up both handlers
print("Opening USB port and establishing connection...\n")
if port.openPort():
    print("Port opened successfully.")
else:
    print("Port failed to open. Exiting.")
    quit()

if port.setBaudRate(57600):
    print("Baud rate set successfully.")
else:
    print("Failed to set baud rate. Exiting.")
    quit()

"""
Next block demonstrates reading from the Dynamixel.

To read, you need to know:
    1. The ID of the motor you are reading from.
    2. The memory address on the motor where the desired data is stored.
    3. The size of the data that you reading: 1, 2 or 4 bytes.

To read, use methods from packetHandler object, match the size:
    read1ByteTxRx
    read2ByteTxRx
    read4ByteTxRx

Parameters:
    1. portHandler object
    2. ID number of motor
    3. memory address on motor

Returns:
    1. Value that was read
    2. Result code (translate to text using getTxRxResult method of 
        packetHandler object), success == COMM_SUCCESS
    3. Error code (translate to text using getRxPacketError method of 
        packetHandler object), success == 0

Most commonly read addresses/sizes (all are read only):
    Present Load (int): 126, 2 bytes (% of max)
    Present Velocity (float): 128, 4 bytes (rpm)
    Present Position (int): 132, 4 bytes (0 = 4096 = home position)
    Present Voltage (int): 144, 2 bytes (ex: 120 = 12.0 V)
    Present Temperature (int): 146, 1 byte (return is in deg C)

Notice that these are based on sensor/encoder data, NOT based on what you 
    wrote.  So, these allow you to compare what is actually happening against
    what you are trying to do.
"""

# Read the ID numbers from the motor memory to test connection.
# Count how many successes to make sure that all are successes.
print("Test reading from each motor:")
read_success = 0
while read_success < len(motors):
    for motor in motors:
        # ID is 1 byte, stored at memory address 7.
        motor_id, result, error = packet_handler.read1ByteTxRx(port, motor, 7)
        if result != COMM_SUCCESS:
            print("Read result was not a success.  The SDK says:")
            print(f"{packet_handler.getTxRxResult(result)} {motor_id} {motor}")
        elif error != 0:
            print("Error found in reading. The SDK says:")
            print(f"{packet_handler.getRxPacketError(error)} {motor_id} {motor}")
        else:
            print(f"Initial connection to motor {motor_id} successful. ID is {motor}")
            read_success += 1
    if read_success < len(motors):
        print("Not all motors succeeded.")
        quit()

"""
Next block demonstrates writing to the Dynamixel, it is very similar to 
reading.  I assume that the connection is working.  Similar code to what
is above would test the connection for each read/write transaction.

To write, you need to know:
    1. The ID of the motor you are writing to.
    2. The memory address on the motor you are writing to.
    3. The size of the data that you writing: 1, 2 or 4 bytes.
    4. The value you are trying to write.

To read, use methods from packetHandler object, match the size:
    write1ByteTxRx
    write2ByteTxRx
    write4ByteTxRx

Parameters (first 3 are same order as read):
    1. portHandler object
    2. ID number of motor
    3. memory address on motor
    4. value you want to write

Returns:
    1. Value that was read
    2. Result code (translate to text using getTxRxResult method of 
        packetHandler object), success == COMM_SUCCESS
    3. Error code (translate to text using getRxPacketError method of 
        packetHandler object), success == 0

NOTE: You can only write to addresses 0-63 if Torque is OFF. The motors will 
        only move if Torque is ON.  So, typical startup (executed below) is:
    1. Set Torque to OFF.
    2. Set Operating Mode.
    3. Set Torque to ON.
    4. Go!

Operating Modes:
    1 = Velocity Control
    3 = Position Control (0-4095 only)
    4 = Extended Position Control (same as 2, but many revolutions allowed)
    16 = PWM control (similar to Velocity but set Goal PWM instead of Goal Vel)

Most commonly written addresses/sizes (all are read/write):
    Operating Mode (int): 11, 1 byte (see above)
    Torque Enable (int): 64, 1 byte (0 = Torque off, 1 = Torque on)
    LED (int): 65, 1 byte (0 = off, 1 = on)
    Goal Velocity (int): 104, 4 bytes (range depends on motor)
        for XL430: -265 to 265
    Goal Position (int): 116, 4 bytes (0-4096 for position mode)
"""

# Set operating mode to extended position
def set_op_mode():
    for motor in motors:
        packet_handler.write1ByteTxRx(port, motor, 64, 0)
        packet_handler.write1ByteTxRx(port, motor, 11, 4)
        packet_handler.write1ByteTxRx(port, motor, 64, 1)
    time.sleep(0.1)

def rest_position():
    # fill in motor_id and position
    packet_handler.write4ByteTxRx(port, MOTOR1_1, 116, 3072)
    packet_handler.write4ByteTxRx(port, MOTOR2_1, 116, 3072)
    packet_handler.write4ByteTxRx(port, MOTOR1_2, 116, 1024)
    packet_handler.write4ByteTxRx(port, MOTOR2_2, 116, 1024)
    packet_handler.write4ByteTxRx(port, MOTOR1_3, 116, 800)
    packet_handler.write4ByteTxRx(port, MOTOR2_3, 116, 800)

def mirror():
    # fill in motor_id for both
    pos,_,_ = packet_handler.read4ByteTxRx(port, MOTOR1_1, 132)
    packet_handler.write4ByteTxRx(port, MOTOR2_1, 116, pos)
    pos,_,_ = packet_handler.read4ByteTxRx(port, MOTOR1_2, 132)
    packet_handler.write4ByteTxRx(port, MOTOR2_2, 116, pos)
    pos,_,_ = packet_handler.read4ByteTxRx(port, MOTOR1_3, 132)
    packet_handler.write4ByteTxRx(port, MOTOR2_3, 116, pos)

def main():
    set_op_mode()
    rest_position()
    time.sleep(2)
    packet_handler.write1ByteTxRx(port, MOTOR1_1, 64, 0)
    packet_handler.write1ByteTxRx(port, MOTOR1_2, 64, 0)
    packet_handler.write1ByteTxRx(port, MOTOR1_3, 64, 0)
    while True:
        mirror()

main()