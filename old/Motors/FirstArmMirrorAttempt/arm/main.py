print("STARTING ARM...")


import time
import dynamixel_sdk

ADDR_ID = 7
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_PRESENT_POS = 132
ADDR_GOAL_POS = 116

OPERATING_MODE = 4 # Extended Position Control Mode

MOTOR_IDS = { # MOTOR_IDS[controller_id] = arm_id
    1: 4,
    2: 5,
    3: 6
}

MOTOR_REST_POSES = {
    1: 3072,
    2: 1024,
    3: 800,
}

port_handler = dynamixel_sdk.PortHandler('/dev/tty.usbserial-FT4THVLF')
packet_handler = dynamixel_sdk.PacketHandler(2.0)

print("Opening USB port and establishing connection...\n")
if port_handler.openPort():
    print("Port opened successfully.")
else:
    print("Port failed to open. Exiting.")
    quit()

if port_handler.setBaudRate(57600):
    print("Baud rate set successfully.")
else:
    print("Failed to set baud rate. Exiting.")
    quit()


def test_motor_id(port_handler, packet_handler, id):
    motor_id, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(port_handler, id, ADDR_ID)
    if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
        print(f"dxl_comm_result error {id} {packet_handler.getTxRxResult(dxl_comm_result)}")
        return False
    elif dxl_error != 0:
        print(f"dxl_error error {id} {packet_handler.getRxPacketError(dxl_error)}")
        return False
    else:
        print(f"{motor_id} active")
        return True

def test_all_motor_ids(port_handler, packet_handler):
    print("Testing each motor:")

    successes = 0
    expected_successes = len(MOTOR_IDS) * 2
    for controller_id, arm_id in MOTOR_IDS.items():
        controller_active = test_motor_id(port_handler, packet_handler, controller_id)
        arm_active        = test_motor_id(port_handler, packet_handler, arm_id)
        successes += int(controller_active) + int(arm_active)
    
    return successes == expected_successes


def set_operating_mode(port_handler, packet_handler, id):
    packet_handler.write1ByteTxRx(port_handler, id, ADDR_TORQUE_ENABLE, 0)
    packet_handler.write1ByteTxRx(port_handler, id, ADDR_OPERATING_MODE, OPERATING_MODE)
    packet_handler.write1ByteTxRx(port_handler, id, ADDR_TORQUE_ENABLE, 1)

def set_all_operating_modes(port_handler, packet_handler):
    for controller_id, arm_id in MOTOR_IDS.items():
        set_operating_mode(port_handler, packet_handler, controller_id)
        set_operating_mode(port_handler, packet_handler, arm_id)


def all_to_rest_poses(port_handler, packet_handler):
    for controller_id, arm_id in MOTOR_IDS.items():
        pos = MOTOR_REST_POSES[controller_id]
        packet_handler.write4ByteTxRx(port_handler, controller_id, ADDR_GOAL_POS, pos)
        packet_handler.write4ByteTxRx(port_handler, arm_id,        ADDR_GOAL_POS, pos)


def all_controller_torque_off(port_handler, packet_handler):
    for controller_id in MOTOR_IDS.keys():
        packet_handler.write1ByteTxRx(port_handler, controller_id, ADDR_TORQUE_ENABLE, 0)

def all_arm_torque_off(port_handler, packet_handler):
    for arm_id in MOTOR_IDS.values():
        packet_handler.write1ByteTxRx(port_handler, arm_id, ADDR_TORQUE_ENABLE, 0)


def mirror(port_handler, packet_handler):
    for controller_id, arm_id in MOTOR_IDS.items():
        controller_pos, _dxl_comm_result, _dxl_error = packet_handler.read4ByteTxRx(port_handler, controller_id, ADDR_PRESENT_POS)
        packet_handler.write4ByteTxRx(port_handler, arm_id, ADDR_GOAL_POS, controller_pos)


def main():
    all_success = test_all_motor_ids(port_handler, packet_handler)
    if not all_success:
        raise RuntimeError("Not all motors activated successful")
    time.sleep(1)

    set_all_operating_modes(port_handler, packet_handler)

    all_to_rest_poses(port_handler, packet_handler)
    time.sleep(1)


    all_controller_torque_off(port_handler, packet_handler)
    time.sleep(1)

    try:
        while True:
            mirror()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        all_arm_torque_off(port_handler, packet_handler)


main()

print("DONE ARM")