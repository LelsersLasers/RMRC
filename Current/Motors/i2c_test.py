import smbus, time
bus = smbus.SMBus(1)

while True: 
    servo = int(input("Servo: "))
    pos = int(input("Position: "))
    bus.write_byte(37, servo)
    bus.write_byte(37, pos)
