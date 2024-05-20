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
    ap.add_argument("-z", "--video-capture-zero", required=False, help="use VideoCapture(0) (testing version)", action="store_true")
    args = vars(ap.parse_args())

    video_capture_zero = args["video_capture_zero"]
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    camera_sqs = {}
    camera_threads = {}

    cap_arg_keys = [None] if video_capture_zero else camera.consts.CAP_ARGS.keys()
    for key in cap_arg_keys:
        camera_sq = util.SingleQueue()
        camera_thread = util.create_thread(camera.main.thread, (camera_sq, key), f"camera_{key}")

        camera_sqs[key] = camera_sq
        camera_threads[key] = camera_thread

    time.sleep(camera.consts.CAMERA_WAKEUP_TIME * 2)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    detection_dq = util.DoubleQueue()
    detection_thread = util.create_thread(detection.main.thread, (detection_dq,), "detection")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    primary_server_dq = util.DoubleQueue()
    primary_server_motor_dq = util.DoubleQueue()
    primary_server_thread = util.create_thread(server.primary_server.main.thread, (primary_server_dq, primary_server_motor_dq), "primary_server")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    arm_server_motor_sq = util.SingleQueue()
    arm_server_thread = util.create_thread(server.arm_server.main.thread, (arm_server_motor_sq,), "arm_server")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    motor_thread = util.create_thread(motors.main.thread, (primary_server_motor_dq, arm_server_motor_sq, video_capture_zero), "motor")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting master thread...\n")

    try:
        master.main.thread(detection_dq, primary_server_dq, camera_sqs, video_capture_zero)
    except Exception as e:
        print("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRROOOOOOOOOOOORRRRRRRRR", e)
        print(traceback.format_exc())
    # ------------------------------------------------------------------------ #


    print("\n\nExiting...\n\n")


    # ------------------------------------------------------------------------ #
    for key in camera_threads.keys():
        camera_thread = camera_threads[key]
        util.close_thread(camera_thread)

    util.close_thread(detection_thread)
    util.close_thread(primary_server_thread)
    util.close_thread(arm_server_thread)
    util.close_thread(motor_thread)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing queues...")

    for camera_sq in camera_sqs.values():
        camera_sq.close()

    detection_dq.close()
    primary_server_dq.close()
    primary_server_motor_dq.close()
    arm_server_motor_sq.close()
    # ------------------------------------------------------------------------ #

    print("Done.")
