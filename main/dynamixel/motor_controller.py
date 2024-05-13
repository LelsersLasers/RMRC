import dynamixel.base_controller
import dynamixel.motor_consts

import time

MOTOR_IDS = { # MOTOR_IDS[side] = [id1, id2]
    "left": [1, 3],
    "right": [2, 4],
}
ORIENTATIONS = { # ORIENTATIONS[side] = direction multiplier
    "left": 1,
    "right": -1,
}


class MotorController(dynamixel.base_controller.BaseController):
    def __init__(self, velocity_limit, min_writes):
        super().__init__()

        self.speeds = { # speeds[side] = %
            "left": 0,
            "right": 0,
        }
        self.statuses = { # statuses[side] = %
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

        # self.velocity_limit = motors.consts.STATE_FROM_SERVER["velocity_limit"]["value"]
        # self.min_writes = motors.consts.STATE_FROM_SERVER["motor_writes"]

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

    def setup_motors(self):
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

                self.set_torque_status(True, motor_id)

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
            self.statuses[side] = 0

            for id in side_ids:
                dxl_present_velocity, _dxl_comm_result, _dxl_error = self.packet_handler.read4ByteTxRx(
                    self.port_handler,
                    id,
                    dynamixel.base_controller.ADDR_PRESENT_VELOCITY
                )
                # adjust for 2's complement
                if dxl_present_velocity > 0x7fffffff:
                    dxl_present_velocity = dxl_present_velocity - 4294967296
                # if dxl_present_current > 0x7fff:
                # 	dxl_present_current = dxl_present_current - 65536
                self.statuses[side] += (dxl_present_velocity / self.velocity_limit * orientation) / 2

                self.check_error_and_maybe_reboot(id)