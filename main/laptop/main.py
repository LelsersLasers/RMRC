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

    base_url = laptop.consts.BASE_TEST_URL if video_capture_zero else laptop.consts.BASE_URL
    have_sent_cycles = False

    try:
        if not video_capture_zero:
            # arm_reader = dynamixel.arm_reader.ArmReader(False)
            # TODO: JUST FOR TESTING, otherwise arm starts off
            arm_reader = dynamixel.arm_reader.ArmReader(True)
            cycles = arm_reader.setup_arm()
        else:
            import random
            frames = 0
            drift = random.randint(-25, 25)
            joint_values = [random.randint(0, 4096) for _ in range(3)]
            cycles = { "j1": 0, "j2": 0, "j3": 0 }
        
        while not graceful_killer.kill_now:
            fps_controller.update()
            fps = fps_controller.fps()   

            if not video_capture_zero:
                arm_reader.update_arm_status()
                j1 = arm_reader.joint_statuses["j1"]
                j2 = arm_reader.joint_statuses["j2"]
                j3 = arm_reader.joint_statuses["j3"]
            else:
                frames += 1
                if frames % laptop.consts.READER_TEST_DRIFT_RESET == 0:
                    drift = random.randint(-25, 25)
                joint_values = [(j + random.randint(-10, 10) + drift) % 4096 for j in joint_values]
                j1, j2, j3 = joint_values

                time.sleep(1 / laptop.consts.READER_TEST_FPS + random.uniform(-0.02, 0.02))

            now = time.time()
            joints_url = base_url + f"joints/{j1}/{j2}/{j3}/{fps}/{now}"
            
            if not have_sent_cycles:
                try:
                    cycles_url = base_url + f"cycles/{cycles['j1']}/{cycles['j2']}/{cycles['j3']}"
                    _response = requests.get(cycles_url, timeout=0.5)
                    have_sent_cycles = True
                except requests.exceptions.RequestException as e:
                    print(f"{type(e)}: {cycles_url}")
                    print("Joints URL:", joints_url)

            if have_sent_cycles:
                try:
                    response = requests.get(joints_url, timeout=0.5)
                    data = response.json()
                    arm_active = data["arm_active"]
                    if not video_capture_zero:
                        arm_reader.maybe_update_torque(arm_active)
                    else:
                        print(f"Arm Active: {arm_active}")
                except requests.exceptions.RequestException as e:
                    print(f"{type(e)}: {joints_url}")
                    time.sleep(laptop.consts.GET_FAIL_WAIT)
    finally:
        if not video_capture_zero:
            print("Closing dynamixel controller...")
            arm_reader.close()
            print("Closed dynamixel controller...")