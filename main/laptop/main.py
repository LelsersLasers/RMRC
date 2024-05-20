# Print before imports for instant feedback
print("STARTING...\n")

import time
import requests

import shared_util

import dynamixel.arm_reader

import laptop.consts


def thread(video_capture_zero):
    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()

    try:
        if not video_capture_zero:
            arm_reader = dynamixel.arm_reader.ArmReader()
            arm_reader.setup_arm_reader()
        else:
            import random
            frames = 0
            drift = random.randint(-10, 10)
            j1    = random.randint(0, 4000)
            j2    = random.randint(0, 4000)
            j3    = random.randint(0, 4000)
        
        while not graceful_killer.kill_now:
            fps_controller.update()
            fps = fps_controller.fps()   

            print(frames)        

            if not video_capture_zero:
                arm_reader.update_arm_status()
                j1 = arm_reader.joint_statuses["j1"]
                j2 = arm_reader.joint_statuses["j2"]
                j3 = arm_reader.joint_statuses["j3"]
            else:
                frames += 1
                if frames % laptop.consts.READER_TEST_DRIFT_RESET == 0:
                    drift = random.randint(-10, 10)
                j1 += random.randint(-10, 10) + drift
                j2 += random.randint(-10, 10) + drift
                j3 += random.randint(-10, 10) + drift

                time.sleep(1 / laptop.consts.READER_TEST_FPS)

            now = time.time()
            arm_url = laptop.consts.ARM_TEST_URL if video_capture_zero else laptop.consts.ARM_URL
            url = arm_url + f"{j1}/{j2}/{j3}/{fps}/{now}"
            try:
                _response = requests.get(url)
            except requests.exceptions.RequestException as e:
                print(f"{type(e)}: {url}")
                time.sleep(laptop.consts.GET_FAIL_WAIT)
    finally:
        if not video_capture_zero:
            print("Closing dynamixel controller...")
            arm_reader.close()
            print("Closed dynamixel controller...")