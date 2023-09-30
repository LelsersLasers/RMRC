cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"


import cv2
import time
import numpy as np
import util

FRAME_PATH = "resources/frame.jpg"
HAZMAT_FRAME_PATH = "resources/hazmat_frame.jpg"
JSON_PATH = "resources/data.json"

QUIT_KEY = "q"
TOGGLE_KEY = "g"
CLEAR_KEY = "c"

FRAME_WRITE_SPACING = 5

STATE = {
    "hazmat_running": False,
    "run_hazmat": False,
    "quit": False,
    "clear_all_found": False,
    "hazmat_delta": 1 / 10,
    "frame_count": 0,
}

# ENUM
class Mode:
    Normal = 0
    Hazmat = 1

    def toggle(state):
        if state == Mode.Normal:
            return Mode.Hazmat
        elif state == Mode.Hazmat:
            return Mode.Normal
        else:
            raise ValueError("Invalid mode")

    def to_str(state):
        if state == Mode.Normal:
            return "Normal"
        elif state == Mode.Hazmat:
            return "Hazmat"
        else:
            raise ValueError("Invalid mode")
        
def main():
    global STATE

    util.write_state(STATE, JSON_PATH)


    print("Starting camera...")

    # cap = cv2.VideoCapture(cap_args, cv2.CAP_GSTREAMER)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError(
            "Can't open camera. Are the cap_args set right? Is the camera plugged in?"
        )


    time.sleep(1)


    print(f"\nPress '{QUIT_KEY}' to quit.")
    print(f"Press '{TOGGLE_KEY}' to toggle running hazmat detection.")
    print(f"Press '{CLEAR_KEY}' to clear all found hazmat labels.\n")

    t0 = time.time()
    t1 = time.time()
    delta = 1 / 30

    mode = Mode.Normal

    last_frame_write = -FRAME_WRITE_SPACING

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Exiting ...")
            break

        STATE = util.read_state(JSON_PATH)

        STATE["frame_count"] += 1

        if not STATE["hazmat_running"] and last_frame_write + FRAME_WRITE_SPACING <= STATE["frame_count"]:
        # if last_frame_write + FRAME_WRITE_SPACING <= STATE["frame_count"]:
        #     while STATE["hazmat_running"]:
        #         STATE = util.read_state(JSON_PATH)
            last_frame_write = STATE["frame_count"]
            cv2.imwrite(FRAME_PATH, frame)

        STATE["run_hazmat"] = mode == Mode.Hazmat

        fps = -1 if delta == 0 else 1 / delta
        hazmat_fps = min(-1 if STATE["hazmat_delta"] == 0 else 1 / STATE["hazmat_delta"], 100)
        print(f"FPS: {fps:.1f}\tHazmat FPS: {hazmat_fps:.1f}\t(Mode: {Mode.to_str(mode)})")

        t1 = time.time()
        delta = t1 - t0
        t0 = t1

        try:
            maybe_hazmat_frame = cv2.imread(HAZMAT_FRAME_PATH)
        except cv2.error as _:
            maybe_hazmat_frame = None

        hazmat_frame = maybe_hazmat_frame if maybe_hazmat_frame is not None else np.zeros_like(frame)
        frame_combined = cv2.hconcat([frame, hazmat_frame])
        cv2.imshow("Camera and hazmat", frame_combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(QUIT_KEY):
            STATE["quit"] = True
            break
        elif key == ord(TOGGLE_KEY):
            mode = Mode.toggle(mode)
        elif key == ord(CLEAR_KEY):
            STATE["clear_all_found"] = True

        util.write_state(STATE, JSON_PATH)

    cap.release()
    cv2.destroyAllWindows()

    STATE["quit"] = True
    util.write_state(STATE, JSON_PATH)

if __name__ == "__main__":
    main()