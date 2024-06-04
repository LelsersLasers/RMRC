import time
import requests
import threading

import shared_util

import dynamixel.arm_reader

import laptop.consts


def thread(no_arm_rest_pos, video_capture_zero):
    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()

    base_url = laptop.consts.BASE_ARM_TEST_URL if video_capture_zero else laptop.consts.BASE_ARM_URL
    have_sent_cycles = False
    last_sent_joints = time.time()
    request_dict = { "arm_active": False }

    try:
        if not video_capture_zero:
            arm_reader = dynamixel.arm_reader.ArmReader()
            cycles = arm_reader.setup_arm(no_arm_rest_pos)
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
                    _response = requests.get(cycles_url, timeout=laptop.consts.GET_TIMEOUT)
                    have_sent_cycles = True
                except requests.exceptions.RequestException as e:
                    print(f"{type(e)}: {cycles_url}")
                    print("Joints URL:", joints_url)

            if have_sent_cycles:
                if request_dict["arm_active"] or now - last_sent_joints > 1 / laptop.consts.ARM_LOW_SEND_RATE:
                    last_sent_joints = now
                    t = threading.Thread(target=joints_request, args=(joints_url, request_dict))
                    t.daemon = True
                    t.start()
            
                if not video_capture_zero:
                    arm_reader.maybe_update_torque(request_dict["arm_active"])
    finally:
        if not video_capture_zero:
            print("Closing dynamixel controller...")
            arm_reader.close()
            print("Closed dynamixel controller...")


def joints_request(joints_url, request_dict):
    try:
        response = requests.get(joints_url, timeout=laptop.consts.GET_TIMEOUT)
        data = response.json()
        request_dict["arm_active"] = data["arm_active"]
    except requests.exceptions.RequestException as e:
        print(f"{type(e)}: {joints_url}")