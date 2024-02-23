import time

import shared_util

import motors.consts
import motors.dynamixel_controller


def thread(server_motor_dq, motor_dq, video_capture_zero):
    fps_controller = shared_util.FPSController()

    server_motor_ds = shared_util.DoubleState(motors.consts.STATE_FROM_SERVER, motors.consts.STATE_FROM_SELF)
    motor_ds = shared_util.DoubleState(motors.consts.STATE_FROM_MASTER, {})
    last_count = motors.consts.STATE_FROM_SERVER["count"]
    last_velocity_count = motors.consts.STATE_FROM_SERVER["velocity_limit"]["count"]


    try:
        if not video_capture_zero:
            dxl_controller = motors.dynamixel_controller.DynamixelController()
            dxl_controller.set_torque_status(True)

        while not motor_ds.s1["quit"]:
            server_motor_ds.update_s1(server_motor_dq)
            motor_ds.update_s1(motor_dq)

            fps_controller.update()
            server_motor_ds.s2["motor_fps"] = fps_controller.fps()

            now = time.time()

            if not video_capture_zero:
                dxl_controller.min_writes = server_motor_ds.s1["motor_writes"]

                # speed calulations use velocity_limit
                velocity_limit_changed = server_motor_ds.s1["velocity_limit"]["count"] > last_velocity_count
                idle_shutoff = now - server_motor_ds.s1["last_get"] > motors.consts.MOTOR_SHUTOFF_TIME
                should_write_velocities = (server_motor_ds.s1["write_every_frame"]
                                    or server_motor_ds.s1["count"] > last_count
                                    or velocity_limit_changed
                                    or idle_shutoff)

                if velocity_limit_changed:
                    last_velocity_count = server_motor_ds.s1["velocity_limit"]["count"]
                    dxl_controller.velocity_limit = server_motor_ds.s1["velocity_limit"]["value"]
                if should_write_velocities:
                    last_count = server_motor_ds.s1["count"]

                    if idle_shutoff:
                        server_motor_ds.s1["left"] = 0
                        server_motor_ds.s1["right"] = 0
                    else:
                        dxl_controller.speeds["left"] = server_motor_ds.s1["left"]
                        dxl_controller.speeds["right"] = server_motor_ds.s1["right"]

                    print(f"Writing speeds: {dxl_controller.speeds}")
                    dxl_controller.update_speeds(dxl_controller.speeds)
                    
                dxl_controller.try_write_speeds()
                dxl_controller.update_status_and_check_errors()

                server_motor_ds.s2["motors"]["target"]  = dxl_controller.speeds
                server_motor_ds.s2["motors"]["current"] = dxl_controller.statuses
            else:
                server_motor_ds.s2["motors"]["target"]["left"]  = server_motor_ds.s1["left"]
                server_motor_ds.s2["motors"]["target"]["right"] = server_motor_ds.s1["right"]

                # just to test
                import random
                ratio_left  = random.random() + 0.5
                ratio_right = random.random() + 0.5
                server_motor_ds.s2["motors"]["current"]["left"]  = server_motor_ds.s1["left"] * ratio_left
                server_motor_ds.s2["motors"]["current"]["right"] = server_motor_ds.s1["right"] * ratio_right

                time.sleep(1 / motors.consts.MOTOR_TEST_FPS)

            server_motor_ds.put_s2(server_motor_dq)
    except KeyboardInterrupt: pass
    finally:
        if not video_capture_zero:
            print("Closing dynamixel controller...")
            # dxl_controller.close()

            dxl_controller.update_speeds({
                "left": 0,
                "right": 0,
            })
            for _ in range(3):
                dxl_controller.try_write_speeds()
            time.sleep(0.25)
            dxl_controller.set_torque_status(False)
            time.sleep(0.25)
            # dxl_controller.port_handler.closePort()
           
            print("Closed dynamixel controller...")