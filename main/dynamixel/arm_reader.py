# Runs on the Laptop

import dynamixel.base_controller
import dynamixel.base_arm
import time

READER_JOINT_IDS = { # READER_JOINT_IDS[joint] = id
    "j1": 1,
    "j2": 2,
    "j3": 3,
}


class ArmReader(dynamixel.base_arm.BaseArm):
    def __init__(self):
        super().__init__(READER_JOINT_IDS)

    def setup_arm_reader(self):
        super().setup_arm()
        time.sleep(dynamixel.base_controller.SHORT_WAIT)
        self.set_torque_status_all(False, self.joint_ids.values())

    def update_arm_status(self):
        for joint, joint_id in self.joint_ids.items():
            pos, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                joint_id,
                dynamixel.base_controller.ADDR_PRESENT_POS
            )
            # adjust for 2's complement
            if pos > 0x7fffffff:
                pos = pos - 4294967296
            # if pos > 0x7fff:
            # 	pos = pos - 65536
            
            self.handle_possible_dxl_issues(joint_id, dxl_comm_result, dxl_error)
            self.check_error_and_maybe_reboot(joint_id)

            self.joint_statuses[joint] = pos