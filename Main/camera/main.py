import time
import cv2
import shared_util
import camera.consts


def thread(camera_dq, key):
    camera_ds = shared_util.DoubleState({}, camera.consts.STATE_FROM_SELF)

    print(f"Opening camera {key}...")
    if key is not None:
        cap = cv2.VideoCapture(camera.consts.CAP_ARGS[key], cv2.CAP_GSTREAMER)
    else:
        cap = cv2.VideoCapture(0)
    print(f"Camera {key} VideoCapture created.")

    if not cap.isOpened():
        raise RuntimeError(f"Can't open camera {key}. Are the cap_args set right? Is the camera plugged in?")
    print(f"Camera {key} opened.")

    time.sleep(camera.consts.CAMERA_WAKEUP_TIME)

    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()

    try:
        while not graceful_killer.kill_now:
            camera_ds.update_s1(camera_dq)

            ret, frame = cap.read()
            if not ret or frame is None:
                print(f"Camera {key} read failed.")
                break

            camera_ds.s2["time"] = time.time()
            camera_ds.s2["frame"] = frame

            fps_controller.update()
            camera_ds.s2["fps"] = fps_controller.fps()

            camera_ds.put_s2(camera_dq)
    finally:
        print(f"Releasing camera {key}...")
        cap.release()