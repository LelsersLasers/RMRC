# Runs on the Robot

import dynamixel.base_controller
import dynamixel.base_arm

OUTPUT_JOINT_IDS = { # OUTPUT_JOINT_IDS[joint] = id
    "j1": 8,
    "j2": 9,
    "j3": 10,
}


class ArmReader(dynamixel.base_arm.BaseArm):
    def __init__(self):
        super().__init__(OUTPUT_JOINT_IDS)

    def update_arm_positions(self, positions):
        for joint, target_pos in positions.items():
            output_joint_id = OUTPUT_JOINT_IDS[joint]

            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler,
                output_joint_id,
                dynamixel.base_controller.ADDR_GOAL_POS,
                target_pos
            )
            self.handle_possible_dxl_issues(output_joint_id, dxl_comm_result, dxl_error)

            read_pos, _dxl_comm_result, _dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                output_joint_id,
                dynamixel.base_controller.ADDR_PRESENT_POS
            )
            # self.handle_possible_dxl_issues(output_joint_id, dxl_comm_result, dxl_error)

            self.check_error_and_maybe_reboot(output_joint_id)
            self.joint_statuses[joint] = read_pos