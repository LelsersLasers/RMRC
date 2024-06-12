import time

import shared_util

import motors.consts
import dynamixel.arm_consts
import server.motor_server.consts

import dynamixel.jetson_controller

import pickle


def process(primary_server_motor_dq, motor_server_motor_dq, no_arm_rest_pos, video_capture_zero):
    primary_server_motor_ds = shared_util.DoubleState(motors.consts.STATE_FROM_SERVER, motors.consts.STATE_FROM_SELF)
    last_count = server.motor_server.consts.STATE_FROM_SELF["count"]
    last_velocity_count = motors.consts.STATE_FROM_SERVER["velocity_limit"]["count"]

    motor_server_motor_ds = shared_util.DoubleState(server.motor_server.consts.STATE_FROM_SELF, server.motor_server.consts.STATE_FROM_MOTORS)
    last_arm_time = server.motor_server.consts.STATE_FROM_SELF["arm_time"]

    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()

    try:
        if not video_capture_zero:
            velocity_limit = primary_server_motor_ds.s1["velocity_limit"]["value"]
            min_writes = primary_server_motor_ds.s1["motor_writes"]
            dxl_controller = dynamixel.jetson_controller.JetsonController(velocity_limit, min_writes)
            dxl_controller.setup(no_arm_rest_pos)
        
        while not graceful_killer.kill_now:
            primary_server_motor_ds.update_s1(primary_server_motor_dq)
            motor_server_motor_ds.update_s1(motor_server_motor_dq)

            fps_controller.update()
            primary_server_motor_ds.s2["motor_fps"] = fps_controller.fps()

            now = time.time()

            # ---------------------------------------------------------------- #
            if not video_capture_zero:
                # ------------------------------------------------------------ #
                dxl_controller.min_writes = primary_server_motor_ds.s1["motor_writes"]

                # speed calulations use velocity_limit
                velocity_limit_changed = primary_server_motor_ds.s1["velocity_limit"]["count"] > last_velocity_count
                idle_shutoff = now - primary_server_motor_ds.s1["last_get"] > motors.consts.MOTOR_SHUTOFF_TIME
                should_write_velocities = (primary_server_motor_ds.s1["write_every_frame"]
                                    or motor_server_motor_ds.s1["count"] > last_count
                                    or velocity_limit_changed
                                    or idle_shutoff)

                if velocity_limit_changed:
                    last_velocity_count = primary_server_motor_ds.s1["velocity_limit"]["count"]
                    dxl_controller.velocity_limit = primary_server_motor_ds.s1["velocity_limit"]["value"]
                if should_write_velocities:
                    last_count = motor_server_motor_ds.s1["count"]

                    if idle_shutoff:
                        motor_server_motor_ds.s1["left"]  = 0
                        motor_server_motor_ds.s1["right"] = 0
                    else:
                        dxl_controller.speeds["left"]  = motor_server_motor_ds.s1["left"]
                        dxl_controller.speeds["right"] = motor_server_motor_ds.s1["right"]

                    display_speeds = {
                        "left":  dxl_controller.speeds["left"]  * motor_server_motor_ds.s1["power_percent"],
                        "right": dxl_controller.speeds["right"] * motor_server_motor_ds.s1["power_percent"],
                    }
                    print(f"Writing speeds: {display_speeds}")

                    dxl_controller.power_percent = motor_server_motor_ds.s1["power_percent"]
                    dxl_controller.update_speeds(dxl_controller.speeds)
                    
                dxl_controller.try_write_speeds()
                dxl_controller.update_motor_status_and_check_errors()

                primary_server_motor_ds.s2["motors"]["target"]  = dxl_controller.speeds
                primary_server_motor_ds.s2["motors"]["current"] = dxl_controller.motor_statuses
                # ------------------------------------------------------------ #

                # ------------------------------------------------------------ #
                new_data = motor_server_motor_ds.s1["arm_time"] > last_arm_time
                arm_active = primary_server_motor_ds.s1["arm_active"]

                arm_target_positions = motor_server_motor_ds.s1["arm_target_positions"]
                arm_cycles = motor_server_motor_ds.s1["cycles"]

                dxl_controller.update_arm_positions(arm_target_positions, arm_cycles, new_data, arm_active)

                arm_target_display = {}
                for joint in arm_target_positions.keys():
                    arm_target_display[joint]  = shared_util.adjust_2s_complement(arm_target_positions[joint])
                    arm_target_display[joint] %= dynamixel.arm_consts.MAX_POSITION

                    dxl_controller.joint_statuses[joint]  = shared_util.adjust_2s_complement(dxl_controller.joint_statuses[joint])
                    dxl_controller.joint_statuses[joint] %= dynamixel.arm_consts.MAX_POSITION
                
                primary_server_motor_ds.s2["arm"]["active"]  = arm_active
                motor_server_motor_ds.s2["arm_active"]       = arm_active

                motor_server_motor_ds.s2["invert"] = primary_server_motor_ds.s1["invert"]
                motor_server_motor_ds.s2["high_send_rate"] = primary_server_motor_ds.s1["high_send_rate"]

                primary_server_motor_ds.s2["arm"]["current"] = dxl_controller.joint_statuses
                
                if new_data:
                    primary_server_motor_ds.s2["arm"]["target"]  = arm_target_display
                    primary_server_motor_ds.s2["arm_reader_fps"] = motor_server_motor_ds.s1["arm_reader_fps"]
                    primary_server_motor_ds.s2["arm_delay"] = time.time() - motor_server_motor_ds.s1["arm_time"] - primary_server_motor_ds.s1["time_offset"]

                    last_arm_time = motor_server_motor_ds.s1["arm_time"]
                # ------------------------------------------------------------ #
            # ---------------------------------------------------------------- #
            else:
                # just to test
                import random
                def rand_ratio(variance):
                    return (random.random() - 0.5) * variance + 1

                # ------------------------------------------------------------ #
                primary_server_motor_ds.s2["motors"]["target"]["left"]  = motor_server_motor_ds.s1["left"]
                primary_server_motor_ds.s2["motors"]["target"]["right"] = motor_server_motor_ds.s1["right"]

                ratio_left  = rand_ratio(0.4)
                primary_server_motor_ds.s2["motors"]["current"]["left"]   = motor_server_motor_ds.s1["left"]
                primary_server_motor_ds.s2["motors"]["current"]["left"]  *= ratio_left
                primary_server_motor_ds.s2["motors"]["current"]["left"]  *= motor_server_motor_ds.s1["power_percent"]

                ratio_right = rand_ratio(0.4)
                primary_server_motor_ds.s2["motors"]["current"]["right"]  = motor_server_motor_ds.s1["right"]
                primary_server_motor_ds.s2["motors"]["current"]["right"] *= ratio_right
                primary_server_motor_ds.s2["motors"]["current"]["right"] *= motor_server_motor_ds.s1["power_percent"]
                # ------------------------------------------------------------ #

                # ------------------------------------------------------------ #
                new_data = motor_server_motor_ds.s1["arm_time"] > last_arm_time

                primary_server_motor_ds.s2["arm"]["active"] = primary_server_motor_ds.s1["arm_active"]
                motor_server_motor_ds.s2["arm_active"]      = primary_server_motor_ds.s1["arm_active"]

                motor_server_motor_ds.s2["invert"] = primary_server_motor_ds.s1["invert"]
                motor_server_motor_ds.s2["high_send_rate"] = primary_server_motor_ds.s1["high_send_rate"]


                if new_data:
                    primary_server_motor_ds.s2["arm_reader_fps"] = motor_server_motor_ds.s1["arm_reader_fps"]
                    primary_server_motor_ds.s2["arm_delay"] = time.time() - motor_server_motor_ds.s1["arm_time"] - primary_server_motor_ds.s1["time_offset"]

                    last_arm_time = motor_server_motor_ds.s1["arm_time"]
                
                arm_target_positions = motor_server_motor_ds.s1["arm_target_positions"]
                primary_server_motor_ds.s2["arm"]["target"]  = arm_target_positions

                for joint in arm_target_positions.keys():
                    ratio = rand_ratio(0.2)
                    primary_server_motor_ds.s2["arm"]["current"][joint] =  arm_target_positions[joint] * ratio
                    primary_server_motor_ds.s2["arm"]["current"][joint] %= dynamixel.arm_consts.MAX_POSITION
                # ------------------------------------------------------------ #

                time.sleep(1 / motors.consts.MOTOR_TEST_FPS +  + random.uniform(-0.02, 0.02))
            # ---------------------------------------------------------------- #

            # Note: directly putting pickled dict into q2 instead of using primary_server_motor_ds.put_s2(dq)
            # Solves issue of left motor value being interpreted as 0 in the server process
            pickled_server_motor_ds_s2 = pickle.dumps(primary_server_motor_ds.s2)
            primary_server_motor_dq.put_q2(pickled_server_motor_ds_s2)

            motor_server_motor_ds.put_s2(motor_server_motor_dq)
    finally:
        if not video_capture_zero:
            print("Closing dynamixel controller...")
            dxl_controller.close()
            print("Closed dynamixel controller...")