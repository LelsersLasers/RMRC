# Goes with ~/Documents/Arduino/serial_test

import serial


num_servos = 6
arduino = serial.Serial('/dev/ttyACM0', baudrate=57600)

def wait_for_start():
    in_byte = arduino.read()
    digit = int.from_bytes(in_byte, 'big') - 48
    if digit == 42:
        return True
    else:
        return False

def get_int():
    my_int = 0
    for i in range(3, -1, -1):
        in_byte = arduino.read()
        digit = int.from_bytes(in_byte, 'big') - 48
        my_int += digit * 10 ** i
    #print(my_int)
    return my_int
    
def getNumServos():
    return num_servos

def updateServos():
    incoming = []
    arduino.reset_input_buffer()
    while not wait_for_start(): pass
    #print("update servos")
    my_int = get_int()
    if my_int == 0:
        incoming.append(my_int)
        for i in range(3):
            incoming.append(get_int())
        if incoming[2] == 1:
            for i in range(2*num_servos - 4):
                incoming.append(get_int())
    return incoming
#    if len(incoming) == 2*num_servos:
#        print(incoming)

