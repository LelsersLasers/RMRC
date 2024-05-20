import time

import shared_util

import motors.consts
import dynamixel.arm_consts
import server.arm_server.consts

import dynamixel.jetson_controller

import pickle


def thread(primary_server_motor_dq, arm_server_motor_sq, video_capture_zero):
    primary_server_motor_ds = shared_util.DoubleState(motors.consts.STATE_FROM_SERVER, motors.consts.STATE_FROM_SELF)
    last_count = motors.consts.STATE_FROM_SERVER["count"]
    last_velocity_count = motors.consts.STATE_FROM_SERVER["velocity_limit"]["count"]

    arm_server_motor_ss = shared_util.SingleState(server.arm_server.consts.STATE_FROM_SELF)

    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()

    try:
        if not video_capture_zero:
            velocity_limit = primary_server_motor_ds.s1["velocity_limit"]["value"]
            min_writes = primary_server_motor_ds.s1["motor_writes"]
            dxl_controller = dynamixel.jetson_controller.JetsonController(velocity_limit, min_writes)
            dxl_controller.setup()
        
        while not graceful_killer.kill_now:
            primary_server_motor_ds.update_s1(primary_server_motor_dq)
            arm_server_motor_ss.update_s(arm_server_motor_sq)

            fps_controller.update()
            primary_server_motor_ds.s2["motor_fps"] = fps_controller.fps()

            now = time.time()

            if not video_capture_zero:
                # ------------------------------------------------------------ #
                dxl_controller.min_writes = primary_server_motor_ds.s1["motor_writes"]

                # speed calulations use velocity_limit
                velocity_limit_changed = primary_server_motor_ds.s1["velocity_limit"]["count"] > last_velocity_count
                idle_shutoff = now - primary_server_motor_ds.s1["last_get"] > motors.consts.MOTOR_SHUTOFF_TIME
                should_write_velocities = (primary_server_motor_ds.s1["write_every_frame"]
                                    or primary_server_motor_ds.s1["count"] > last_count
                                    or velocity_limit_changed
                                    or idle_shutoff)

                if velocity_limit_changed:
                    last_velocity_count = primary_server_motor_ds.s1["velocity_limit"]["count"]
                    dxl_controller.velocity_limit = primary_server_motor_ds.s1["velocity_limit"]["value"]
                if should_write_velocities:
                    last_count = primary_server_motor_ds.s1["count"]

                    if idle_shutoff:
                        primary_server_motor_ds.s1["left"] = 0
                        primary_server_motor_ds.s1["right"] = 0
                    else:
                        dxl_controller.speeds["left"] = primary_server_motor_ds.s1["left"]
                        dxl_controller.speeds["right"] = primary_server_motor_ds.s1["right"]

                    print(f"Writing speeds: {dxl_controller.speeds}")
                    dxl_controller.update_speeds(dxl_controller.speeds)
                    
                dxl_controller.try_write_speeds()
                dxl_controller.update_motor_status_and_check_errors()

                primary_server_motor_ds.s2["motors"]["target"]  = dxl_controller.speeds
                primary_server_motor_ds.s2["motors"]["current"] = dxl_controller.motor_statuses
                # ------------------------------------------------------------ #

                # ------------------------------------------------------------ #
                arm_active = primary_server_motor_ds.s1["arm_active"]
                arm_target_positions = arm_server_motor_ss.s["arm_target_positions"]
                dxl_controller.update_arm_positions(arm_target_positions, arm_active)

                for joint in arm_target_positions.keys():
                    arm_target_positions[joint] =  shared_util.adjust_2s_complement(arm_target_positions[joint])
                    arm_target_positions[joint] %= dynamixel.arm_consts.MAX_POSITION

                    dxl_controller.joint_statuses[joint] = shared_util.adjust_2s_complement(dxl_controller.joint_statuses[joint])
                    dxl_controller.joint_statuses[joint] %= dynamixel.arm_consts.MAX_POSITION
                
                primary_server_motor_ds.s2["arm"]["active"]  = arm_active
                primary_server_motor_ds.s2["arm"]["target"]  = arm_target_positions
                primary_server_motor_ds.s2["arm"]["current"] = dxl_controller.joint_statuses
                primary_server_motor_ds.s2["arm_reader_fps"] = arm_server_motor_ss.s["arm_reader_fps"]
                primary_server_motor_ds.s2["arm_delay"] = time.time() - arm_server_motor_ss.s["time"]
                # ------------------------------------------------------------ #
            else:
                # just to test
                import random
                def rand_ratio(variance):
                    return (random.random() - 0.5) * variance + 1

                # ------------------------------------------------------------ #
                primary_server_motor_ds.s2["motors"]["target"]["left"]  = primary_server_motor_ds.s1["left"]
                primary_server_motor_ds.s2["motors"]["target"]["right"] = primary_server_motor_ds.s1["right"]

                ratio_left  = rand_ratio(0.4)
                ratio_right = rand_ratio(0.4)
                primary_server_motor_ds.s2["motors"]["current"]["left"]  = primary_server_motor_ds.s1["left"] * ratio_left
                primary_server_motor_ds.s2["motors"]["current"]["right"] = primary_server_motor_ds.s1["right"] * ratio_right
                # ------------------------------------------------------------ #

                # ------------------------------------------------------------ #
                primary_server_motor_ds.s2["arm"]["active"] = primary_server_motor_ds.s1["arm_active"]
                primary_server_motor_ds.s2["arm_reader_fps"] = arm_server_motor_ss.s["arm_reader_fps"]
                primary_server_motor_ds.s2["arm_delay"] = time.time() - arm_server_motor_ss.s["time"]
                
                arm_target_positions = arm_server_motor_ss.s["arm_target_positions"]
                primary_server_motor_ds.s2["arm"]["target"]  = arm_target_positions

                for joint in arm_target_positions.keys():
                    ratio = rand_ratio(0.2)
                    primary_server_motor_ds.s2["arm"]["current"][joint] =  arm_target_positions[joint] * ratio
                    primary_server_motor_ds.s2["arm"]["current"][joint] %= dynamixel.arm_consts.MAX_POSITION
                # ------------------------------------------------------------ #

                time.sleep(1 / motors.consts.MOTOR_TEST_FPS)

            # Note: directly putting pickled dict into q2 instead of using primary_server_motor_ds.put_s2(dq)
            # Solves issue of left motor value being interpreted as 0 in the server thread
            pickled_server_motor_ds_s2 = pickle.dumps(primary_server_motor_ds.s2)
            primary_server_motor_dq.put_q2(pickled_server_motor_ds_s2)
    finally:
        if not video_capture_zero:
            print("Closing dynamixel controller...")
            dxl_controller.close()
            print("Closed dynamixel controller...")