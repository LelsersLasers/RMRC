# Print before imports for instant feedback
print("STARTING...\n")

import time
import argparse
import requests

import shared_util

import dynamixel.arm_reader

import laptop.consts


BASE_URL = "127.0.0.1:5000"
ARM_URL = BASE_URL + "/arm/" # + j1 + "/" + j2 + "/" + j3 + "/" + fps + "/" + time


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
            fps = fps_controller.fps()           

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

            now = time.time()
            if now - last_send < 1 / laptop.consts.SEND_RATE:
                continue

            last_send = now
            url = ARM_URL + f"{j1}/{j2}/{j3}/{fps}/{now}"
            # _response = requests.get(url)
            print(url)
    finally:
        if not video_capture_zero:
            print("Closing dynamixel controller...")
            arm_reader.close()
            print("Closed dynamixel controller...")