import dynamixel_sdk

DEVICE_NAME = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ORIENTATIONS = [1, -1, -1, 1]

class DynamixelController:
	def __init__(self):
		self.port_handler = dynamixel_sdk.PortHandler(DEVICE_NAME)
		self.packet_handler = dynamixel_sdk.PacketHandler(PROTOCOL_VERSION)
		
		if self.port_handler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port.")
		if self.port_handler.setBaudRate(BAUDRATE):
			print("Succeeded to change the baudrate")
		else:
			print("Failed to change the baudrate.")

		self.speeds = [0, 0, 0, 0]
		self.ids = [1, 2, 3, 4]

	def reset_motors(self):
		for id in self.ids:
			self.packet_handler.reboot(self.port_handler, id)

	def close_port(self):
		self.port_handler.closePort()

	def set_torque_status(self, status):
		status_id = 1 if status else 0
		for id in self.ids:
			dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, id, ADDR_TORQUE_ENABLE, status_id)
			if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
				print(f"dxl_comm_result error {id} {self.packet_handler.getTxRxResult(dxl_comm_result)}")
			elif dxl_error != 0:
				print(f"dxl_error error {id} {self.packet_handler.getRxPacketError(dxl_error)}")

	def update_speed(self):
		for id, speed, orientation in zip(self.ids, self.speeds, ORIENTATIONS):
			dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, id, ADDR_GOAL_VELOCITY, speed * orientation)
			if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
				print(f"dxl_comm_result error {id} {self.packet_handler.getTxRxResult(dxl_comm_result)}")
			elif dxl_error != 0:
				print(f"dxl_error error {id} {self.packet_handler.getRxPacketError(dxl_error)}")


"""
drive_4x=dxl_4x()
drive_4x.torqueOn()

	drive_4x.speedMotor1 = arrNum[0]
	drive_4x.speedMotor2 = arrNum[1]
	drive_4x.speedMotor3 = arrNum[2]
	drive_4x.speedMotor4 = arrNum[3]
	drive_4x.reset = arrNum[4]
	drive_4x.updateSpeed()

drive_4x.torqueOff()
drive_4x.closePort()
"""


"""
import smbus

bus = smbus.SMBus(1)
bus2 = smbus.SMBus(8)


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