# Runs on the Laptop

import dynamixel.base_controller
import dynamixel.base_arm

READER_JOINT_IDS = { # READER_JOINT_IDS[joint] = id
    "j1": 5,
    "j2": 6,
    "j3": 7,
}


class ArmReader(dynamixel.base_arm.BaseArm):
    def __init__(self):
        super().__init__(READER_JOINT_IDS)

    def setup_arm_reader(self):
        super().setup_arm()
        self.set_torque_status_all(False, self.joint_ids.values())

    def update_arm_status(self):
        for joint, joint_id in self.joint_ids.items():
            pos, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                joint_id,
                dynamixel.base_controller.ADDR_PRESENT_POS
            )
            if dxl_present_velocity > 0x7fffffff:
                dxl_present_velocity = dxl_present_velocity - 4294967296
            # if dxl_present_current > 0x7fff:
            # 	dxl_present_current = dxl_present_current - 65536
            self.handle_possible_dxl_issues(joint_id, dxl_comm_result, dxl_error)
            self.check_error_and_maybe_reboot(joint_id)

            self.joint_statuses[joint] = pos