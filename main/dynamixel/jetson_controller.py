# Runs on the Robot

import shared_util

import dynamixel.base_arm
import dynamixel.base_controller
import dynamixel.motor_consts

import time


OUTPUT_JOINT_IDS = { # OUTPUT_JOINT_IDS[joint] = id
    "j1": 8,
    "j2": 9,
    "j3": 10,
}

MOTOR_IDS = { # MOTOR_IDS[side] = [id1, id2]
    "left": [1, 3],
    "right": [2, 4],
}
ORIENTATIONS = { # ORIENTATIONS[side] = direction multiplier
    "left": 1,
    "right": -1,
}


class JetsonController(dynamixel.base_arm.BaseArm):
    def __init__(self, velocity_limit, min_writes):
        super().__init__(OUTPUT_JOINT_IDS)

        self.speeds = { # speeds[side] = %
            "left": 0,
            "right": 0,
        }
        self.motor_statuses = { # motor_statuses[side] = %
            "left": 0,
            "right": 0,
        }
        self.to_writes = { # to_writes[id] = #
            1: 0,
            2: 0,
            3: 0,
            4: 0,
        }
        self.has_wrote = { # has_wrote[id] = #
            1: 0,
            2: 0,
            3: 0,
            4: 0,
        }
        self.velocity_limit = velocity_limit
        self.min_writes = min_writes
        self.cycles = {} # cycles[joint] = pos // 4096

    def close(self):
        self.update_speeds({
            "left": 0,
            "right": 0,
        })
        for _ in range(dynamixel.motor_consts.MAX_WRITES):
            self.try_write_speeds()
        time.sleep(dynamixel.base_controller.SHORT_WAIT)

        for motor_ids in MOTOR_IDS.values():
            self.set_torque_status_all(False, motor_ids)
        time.sleep(dynamixel.base_controller.SHORT_WAIT)
        
        super().close()

    def setup(self):
        for motor_ids in MOTOR_IDS.values():
            self.set_torque_status_all(False, motor_ids)
        time.sleep(dynamixel.base_controller.SHORT_WAIT)

        for motor_ids in MOTOR_IDS.values():
            for motor_id in motor_ids:
                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler,
                    motor_id,
                    dynamixel.base_controller.ADDR_OPERATING_MODE,
                    dynamixel.base_controller.MOTOR_OPERATING_MODE
                )
                self.handle_possible_dxl_issues(motor_id, dxl_comm_result, dxl_error)

                self.check_error_and_maybe_reboot(motor_id, True)
                time.sleep(dynamixel.base_controller.SHORT_WAIT)

                self.set_torque_status(True, motor_id)
        
        self.cycles = super().setup_arm()

    def update_speeds(self, speeds):
        self.speeds = speeds
        for id in self.to_writes:
            self.to_writes[id] = self.min_writes
            self.has_wrote[id] = 0

    def try_write_speeds(self):
        for side, side_ids in MOTOR_IDS.items():
            speed = self.speeds[side]
            orientation = ORIENTATIONS[side]
            power = int(speed * orientation * self.velocity_limit)

            for id in side_ids:
                if self.to_writes[id] > 0 and self.has_wrote[id] < dynamixel.motor_consts.MAX_WRITES:
                    dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                        self.port_handler,
                        id,
                        dynamixel.base_controller.ADDR_GOAL_VELOCITY,
                        power
                    )
                    success = self.handle_possible_write_issues(id, dxl_comm_result, dxl_error)
                    if success:
                        self.to_writes[id] -= 1
                    self.has_wrote[id] += 1
    
    def update_motor_status_and_check_errors(self):
        for side, side_ids in MOTOR_IDS.items():
            orientation = ORIENTATIONS[side]
            self.motor_statuses[side] = 0

            for id in side_ids:
                dxl_present_velocity, _dxl_comm_result, _dxl_error = self.packet_handler.read4ByteTxRx(
                    self.port_handler,
                    id,
                    dynamixel.base_controller.ADDR_PRESENT_VELOCITY
                )
                dxl_present_velocity = shared_util.adjust_2s_complement(dxl_present_velocity)
                self.motor_statuses[side] += (dxl_present_velocity / self.velocity_limit * orientation) / 2

                self.check_error_and_maybe_reboot(id)
    
    def update_arm_positions(self, target_positions, reader_cycles, active):
        for joint, target_pos in target_positions.items():
            output_joint_id = OUTPUT_JOINT_IDS[joint]

            if active:
                adjusted_target_pos = (self.cycles[joint] - reader_cycles[joint]) * 4096 + target_pos

                dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                    self.port_handler,
                    output_joint_id,
                    dynamixel.base_controller.ADDR_GOAL_POS,
                    adjusted_target_pos
                )
                self.handle_possible_dxl_issues(output_joint_id, dxl_comm_result, dxl_error)

            read_pos, _dxl_comm_result, _dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                output_joint_id,
                dynamixel.base_controller.ADDR_PRESENT_POS
            )
            if not active:
                self.handle_possible_dxl_issues(output_joint_id, dxl_comm_result, dxl_error)

            self.check_error_and_maybe_reboot(output_joint_id)
            self.joint_statuses[joint] = read_pos