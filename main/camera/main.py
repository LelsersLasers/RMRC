import time
import cv2
import shared_util
import camera.consts


def thread(camera_sq, key):
    camera_ss = shared_util.SingleState(camera.consts.STATE_FROM_SELF)

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
            ret, frame = cap.read()
            if not ret or frame is None:
                print(f"Camera {key} read failed.")
                time.sleep(camera.consts.CAMERA_WAIT_AFTER_FAIL)

            if key is None and frame is not None:
                frame = cv2.resize(frame, camera.consts.CAMERA_SIZE)

            camera_ss.s["time"] = time.time()
            camera_ss.s["frame"] = frame

            fps_controller.update()
            camera_ss.s["fps"] = fps_controller.fps()

            camera_ss.put_s(camera_sq)
    finally:
        print(f"Releasing camera {key}...")
        cap.release()