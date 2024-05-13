# Print before imports for instant feedback
print("STARTING...\n")

import time
import argparse
import requests

import shared_util

import dynamixel.arm_reader

import laptop.consts


BASE_URL = "127.0.0.1:5000"
ARM_URL = BASE_URL + "/arm/" # + j1 + "/" + j2 + "/" + j3 + "/" + fps


def thread(video_capture_zero):
    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()
    last_send = time.time()

    try:
        if not video_capture_zero:
            arm_reader = dynamixel.arm_reader.ArmReader()
            arm_reader.setup_arm_reader()
        
        while not graceful_killer.kill_now:
            fps_controller.update()
            fps = fps_controller.get_fps()           

            if not video_capture_zero:
                arm_reader.update_arm_status()
                j1 = arm_reader.joint_statuses["j1"]
                j2 = arm_reader.joint_statuses["j2"]
                j3 = arm_reader.joint_statuses["j3"]
            else:
                import random
                j1 = random.randint(0, 440)
                j2 = random.randint(0, 440)
                j3 = random.randint(0, 440)

                time.sleep(1 / laptop.consts.READER_TEST_FPS)

            if time.time() - last_send < 1 / laptop.consts.SEND_FPS:
                continue

            last_send = time.time()
            url = ARM_URL + f"{j1}/{j2}/{j3}/{fps}"
            _response = requests.get(url)
    finally:
        if not video_capture_zero:
            print("Closing dynamixel controller...")
            arm_reader.close_arm_reader()
            print("Closed dynamixel controller...")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-z", "--video-capture-zero", required=False, help="use VideoCapture(0)", action="store_true")
    args = vars(ap.parse_args())

    video_capture_zero = args["video_capture_zero"]

    thread(video_capture_zero)

    print("Exiting...")