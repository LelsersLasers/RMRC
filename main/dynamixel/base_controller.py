import dynamixel_sdk

# ---------------------------------------------------------------------------- #
# https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
# https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/python/python_porthandler/#python

DEVICE_NAME = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0

BAUDRATE = 57600

ARM_OPERATING_MODE    = 4 # Extended Position Control Mode
MOTOR_OPERATING_MODE  = 1 # Velocity Control Mode

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_ERROR_CODE       = 70
ADDR_GOAL_VELOCITY    = 104
ADDR_GOAL_POS         = 116
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POS      = 132

SHORT_WAIT = 0.1
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class BaseController:
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

    def close(self):
        self.port_handler.closePort()

    def handle_possible_dxl_issues(self, id, dxl_comm_result, dxl_error):
        if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
            print(f"dxl_comm_result error {id} {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"dxl_error error {id} {self.packet_handler.getRxPacketError(dxl_error)}")
            return False
        else:
            return True

    def set_torque_status(self, status, id):
        status_code = 1 if status else 0
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, id, ADDR_TORQUE_ENABLE, status_code)
        self.handle_possible_dxl_issues(id, dxl_comm_result, dxl_error)

    def set_torque_status_all(self, status, ids):
        for id in ids: self.set_torque_status(status, id)

    def check_error_and_maybe_reboot(self, id):
        error_code, _dxl_comm_result, _dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, id, ADDR_ERROR_CODE)
        if error_code > 0:
            print(f"error_code {id} {error_code} {self.packet_handler.getRxPacketError(error_code)}")
            print(f"rebooting {id}")
            self.packet_handler.reboot(self.port_handler, id)
# ---------------------------------------------------------------------------- #