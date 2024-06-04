# Print before imports for instant feedback
print("STARTING...\n")

import time
import argparse
import traceback

import util

import detection.consts
import detection.main

import motors.consts
import motors.main

import server.arm_server.main
import server.primary_server.main

import camera.consts
import camera.main

import master.main


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-a", "--no-arm-rest-pos", required=False, help="don't move the arm to default rest position", action="store_true")
    ap.add_argument("-z", "--video-capture-zero", required=False, help="use VideoCapture(0) (testing version)", action="store_true")
    args = vars(ap.parse_args())

    no_arm_rest_pos = args["no_arm_rest_pos"]
    video_capture_zero = args["video_capture_zero"]
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    camera_sqs = {}
    camera_processes = {}

    cap_arg_keys = [None] if video_capture_zero else camera.consts.CAP_ARGS.keys()
    for key in cap_arg_keys:
        camera_sq = util.SingleQueue()
        camera_process = util.create_process(camera.main.process, (camera_sq, key), f"camera_{key}")

        camera_sqs[key] = camera_sq
        camera_processes[key] = camera_process

    time.sleep(camera.consts.CAMERA_WAKEUP_TIME * 2)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    detection_dq = util.DoubleQueue()
    detection_process = util.create_process(detection.main.process, (detection_dq,), "detection")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    primary_server_dq = util.DoubleQueue()
    primary_server_motor_dq = util.DoubleQueue()
    primary_server_process = util.create_process(server.primary_server.main.process, (primary_server_dq, primary_server_motor_dq), "primary_server")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    arm_server_motor_dq = util.DoubleQueue()
    arm_server_process = util.create_process(server.arm_server.main.process, (arm_server_motor_dq,), "arm_server")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    motor_process = util.create_process(motors.main.process, (primary_server_motor_dq, arm_server_motor_dq, no_arm_rest_pos, video_capture_zero), "motor")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting master process...\n")

    try:
        master.main.process(detection_dq, primary_server_dq, camera_sqs, video_capture_zero)
    except Exception as e:
        print("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRROOOOOOOOOOOORRRRRRRRR", e)
        print(traceback.format_exc())
    # ------------------------------------------------------------------------ #


    print("\n\nExiting...\n\n")


    # ------------------------------------------------------------------------ #
    for key in camera_processes.keys():
        camera_process = camera_processes[key]
        util.close_process(camera_process)

    util.close_process(detection_process)
    util.close_process(primary_server_process)
    util.close_process(arm_server_process)
    util.close_process(motor_process)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing queues...")

    for camera_sq in camera_sqs.values():
        camera_sq.close()

    detection_dq.close()
    primary_server_dq.close()
    primary_server_motor_dq.close()
    arm_server_motor_dq.close()
    # ------------------------------------------------------------------------ #

    print("Done.")
