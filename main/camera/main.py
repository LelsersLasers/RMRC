import time
import cv2
import shared_util
import camera.consts


def process(camera_dq, key):
    camera_ds = shared_util.DoubleState(camera.consts.STATE_FROM_MASTER, camera.consts.STATE_FROM_SELF)

    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()

    cap = None

    try:
        while not graceful_killer.kill_now:
            camera_ds.update_s1(camera_dq)

            fps_controller.update()
            camera_ds.s2["fps"] = fps_controller.fps()
            
            if cap is None and (key in camera_ds.s1["active_keys"] or key is None):
                # not open and should be open
                print(f"Opening camera {key}...")
                time.sleep(camera.consts.CAMERA_WAIT_AFTER_ACTIVE)
                if key is not None:
                    cap = cv2.VideoCapture(camera.consts.CAP_ARGS[key], cv2.CAP_GSTREAMER)
                else:
                    cap = cv2.VideoCapture(0)
                print(f"Camera {key} VideoCapture created.")

                if not cap.isOpened():
                    print(f"Can't open camera {key}. Are the cap_args set right? Is the camera plugged in?")
                    cap.release()
                    cap = None
                    time.sleep(camera.consts.CAMERA_WAIT_AFTER_FAIL)
                    continue
                print(f"Camera {key} opened.")

                time.sleep(camera.consts.CAMERA_WAKEUP_TIME)
            elif cap is not None and (key in camera_ds.s1["active_keys"] or key is None):
                # open and should be open
                ret, frame = cap.read()
                if not ret or frame is None:
                    print(f"Camera {key} read failed.")
                    print(f"Releasing camera {key}...")
                    cap.release()
                    cap = None
                    time.sleep(camera.consts.CAMERA_WAIT_AFTER_FAIL)

                if key is None:
                    frame = cv2.resize(frame, camera.consts.CAMERA_SIZE)

                camera_ds.s2["time"] = time.time()
                camera_ds.s2["frame"] = frame
            elif cap is not None and key not in camera_ds.s1["active_keys"]:
                # open and should be closed
                print(f"Releasing camera {key}...")
                cap.release()
                cap = None
            elif cap is None and key not in camera_ds.s1["active_keys"]:
                # not open and should be closed
                time.sleep(1 / camera.consts.DRY_FPS)

            camera_ds.put_s2(camera_dq)
    finally:
        if cap is not None:
            print(f"Releasing camera {key}...")
            cap.release()