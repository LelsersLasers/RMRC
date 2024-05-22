import dynamixel.base_controller
import dynamixel.arm_consts

import time


class BaseArm(dynamixel.base_controller.BaseController):
    def __init__(self, joint_ids):
        super().__init__()
        self.joint_ids = joint_ids # joint_ids[joint] = id
        self.joint_statuses = { # joint_statuses[joint] = #
            "j1": 0,
            "j2": 0,
            "j3": 0,
        }

    def close(self):
        self.set_torque_status_all(False, self.joint_ids.values())
        time.sleep(dynamixel.base_controller.SHORT_WAIT)
        super().close()

    def setup_arm(self):
        self.set_torque_status_all(False, self.joint_ids.values())
        time.sleep(dynamixel.base_controller.SHORT_WAIT)

        starting_poses = {} # starting_poses[joint]: (pos, pos % dynamixel.arm_consts.MAX_POSITION)

        for joint, joint_id in self.joint_ids.items():
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                joint_id,
                dynamixel.base_controller.ADDR_OPERATING_MODE,
                dynamixel.base_controller.ARM_OPERATING_MODE
            )
            self.handle_possible_dxl_issues(joint_id, dxl_comm_result, dxl_error)

            pos, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                joint_id,
                dynamixel.base_controller.ADDR_PRESENT_POS
            )
            self.handle_possible_dxl_issues(joint_id, dxl_comm_result, dxl_error)

            self.set_torque_status(True, joint_id)

            starting_poses[joint] = (pos, pos % dynamixel.arm_consts.MAX_POSITION)

        # Have to make sure joints don't collide with the base plate or robot

        if 1024 < starting_poses["j2"][1] < 3072:
            joint_order = ["j1", "j2", "j3"]
        else:
            joint_order = ["j1", "j3", "j2"]

        cycles = {} # cycles[joint] = pos // 4096

        for joint in joint_order:
            joint_id = self.joint_ids[joint]

            base_rest_pos = dynamixel.arm_consts.ARM_REST_POSES[joint]
            low_rest_pos = 4096 * (starting_poses["j2"][0] // 4096) + base_rest_pos
            high_rest_pos = low_rest_pos + 4096

            if joint == "j1":
                low_dist  = abs(starting_poses[joint][1] - low_rest_pos)
                high_dist = abs(starting_poses[joint][1] - high_rest_pos)
                if low_dist <= high_dist:
                    rest_pos = low_rest_pos
                else:
                    rest_pos = high_rest_pos
            elif joint == "j2":
                # j2 can not cross 2048 which is straight down as it moves to rest position
                if 2048 < starting_poses["j2"][1] < base_rest_pos:
                    rest_pos = high_rest_pos
                else:
                    rest_pos = low_rest_pos
            elif joint == "j3":
                rest_pos = low_rest_pos


            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler,
                joint_id,
                dynamixel.base_controller.ADDR_GOAL_POS,
                rest_pos
            )
            self.handle_possible_dxl_issues(joint_id, dxl_comm_result, dxl_error)
            self.check_error_and_maybe_reboot(joint_id)

            time.sleep(dynamixel.base_controller.SHORT_WAIT)

        return cycles
            
