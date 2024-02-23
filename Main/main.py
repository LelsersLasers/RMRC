import time
import argparse
import traceback

import util

import hazmat.consts
import hazmat.main

import motors.consts
import motors.main

import server.consts
import server.main

import camera.consts
import camera.main

import master.consts
import master.main


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-z", "--video-capture-zero", required=False, help="use VideoCapture(0)", action="store_true")
    args = vars(ap.parse_args())

    video_capture_zero = args["video_capture_zero"]
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    camera_dqs = {}
    camera_threads = {}

    cap_arg_keys = [None] if video_capture_zero else camera.consts.CAP_ARGS.keys()
    for key in cap_arg_keys:
        camera_dq = util.DoubleQueue()
        camera_thread = util.create_thread(camera.main.thread, (camera_dq, key), f"camera_{key}")

        camera_dqs[key] = camera_dq
        camera_threads[key] = camera_thread

    time.sleep(camera.consts.CAMERA_WAKEUP_TIME * 2)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    hazmat_dq = util.DoubleQueue()
    hazmat_thread = util.create_thread(hazmat.main.thread, (hazmat_dq,), "hazmat")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    server_dq = util.DoubleQueue()
    server_motor_dq = util.DoubleQueue()
    flask_thread = util.create_thread(server.main.thread, (server_dq, server_motor_dq), "flask")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    motor_dq = util.DoubleQueue()
    motor_thread = util.create_thread(motors.main.thread, (server_motor_dq, motor_dq, video_capture_zero), "motor")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting master thread...\n")

    try:
        gpu_log_file = None if video_capture_zero else open(master.consts.GPU_LOG_FILENAME, 'rb')
        master.main.thread(hazmat_dq, server_dq, camera_dqs, video_capture_zero, gpu_log_file)
    except Exception as e:
        print("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRROOOOOOOOOOOORRRRRRRRR", e)
        print(traceback.format_exc())
    # except: pass
    finally:
        if not video_capture_zero:
            gpu_log_file.close()
    # ------------------------------------------------------------------------ #


    print("\n\nExiting...\n\n")


    # ------------------------------------------------------------------------ #
    for key in camera_threads.keys():
        camera.consts.STATE_FROM_MASTER["quit"] = True
        camera_dq = camera_dqs[key]
        camera_dq.put_q1(camera.consts.STATE_FROM_MASTER)
        camera_thread = camera_threads[key]
        util.close_thread(camera_thread)

    hazmat.consts.STATE_FROM_MASTER["quit"] = True
    hazmat_dq.put_q1(hazmat.consts.STATE_FROM_MASTER)
    util.close_thread(hazmat_thread)

    util.close_thread(flask_thread)

    motors.consts.STATE_FROM_MASTER["quit"] = True
    motor_dq.put_q1(motors.consts.STATE_FROM_MASTER)
    util.close_thread(motor_thread)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing queues...")

    hazmat_dq.close()
    server_dq.close()
    server_motor_dq.close()

    for camera_dq in camera_dqs.values():
        camera_dq.close()
    # ------------------------------------------------------------------------ #

    print("Done.")
