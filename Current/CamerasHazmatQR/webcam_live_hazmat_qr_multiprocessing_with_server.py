cap_args = {
    "webcam1": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "webcam2": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "ir": "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! appsink",
}


import cv2
import time
import numpy as np
import argparse
from multiprocessing import Process, Queue
import argparse
import json
import base64
import queue
import util
import hazmat
import qr_detect


HAZMAT_TOGGLE_KEY = "h"
HAZMAT_HOLD_KEY = "g"
QR_TOGGLE_KEY = "r"
HAZMAT_CLEAR_KEY = "c"
QR_CLEAR_KEY = "x"

CAMERA_WAKEUP_TIME = 0.5
HAZMAT_FRAME_SCALE = 1
HAZMAT_DELAY_BAR_SCALE = 30 # in seconds
QR_TIME_BAR_SCALE = 0.1 # in seconds
SERVER_FRAME_SCALE = 1

#------------------------------------------------------------------------------#
# What main thread sends
START_STATE_MAIN = {
    "frame": None,
    "run_hazmat": False,
    "quit": False,
    "clear_all_found": 0,
}

# What hazmat thread sends
START_STATE_HAZMAT = {
    "hazmat_fps": 100,
    "hazmat_frame": None,
    "hazmats_found": [],
}
#------------------------------------------------------------------------------#


#------------------------------------------------------------------------------#
MAIN_FILE = "states/state.json"
SERVER_FILE = "states/server_state.json"
FILE_DELAY = 0.0

MAIN_STATE = {
    "frame": "",
    "w": 1,
    "h": 1,
    "hazmats_found": [],
    "qr_found": [],
}
SERVER_STATE = {}

def write_state():
    # Write state to file
    with open(MAIN_FILE, "w") as f:
        json.dump(MAIN_STATE, f)

def read_state():
	global SERVER_STATE
	try:
		with open(SERVER_FILE, "r") as f:
			SERVER_STATE = json.load(f)
	except:
		time.sleep(FILE_DELAY)
		read_state()
#------------------------------------------------------------------------------#


#------------------------------------------------------------------------------#
def hazmat_main(main_queue, hazmat_queue):
    time.sleep(CAMERA_WAKEUP_TIME)

    fps_controller = util.FPS()

    all_found = []
    frame = None

    state_main = START_STATE_MAIN
    state_hazmat = START_STATE_HAZMAT

    while not state_main["quit"]:

        clear_all_found = False
        while True:
            try:
                state_main = main_queue.get_nowait()
                clear_all_found = clear_all_found or state_main["clear_all_found"] > 0
            except queue.Empty:
                break

        if clear_all_found:
            all_found = []
            print("Cleared all found hazmat labels.")

        if state_main["frame"] is not None:
            frame = state_main["frame"]
            frame = cv2.resize(frame, (0, 0), fx=HAZMAT_FRAME_SCALE, fy=HAZMAT_FRAME_SCALE)

            if state_main["run_hazmat"]:

                threshVals = [90, 100, 110, 120, 130, 140, 150, 160, 170]
                found_this_frame = []
                for threshVal in threshVals:
                    received_tups = hazmat.processScreenshot(frame, threshVal)

                    for r in received_tups:
                        found_this_frame.append(r)
                        all_found.append(r[0].strip())

                found_this_frame = util.remove_dups(found_this_frame, lambda x: x[0])

                fontScale = 0.5
                fontColor = (0, 0, 255)
                thickness = 1
                lineType = 2

                for found in found_this_frame:
                    text, cnt = found

                    frame = cv2.drawContours(frame, [cnt], -1, (255, 0, 0), 3)
                    x, y, w, h = cv2.boundingRect(cnt)

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (225, 0, 0), 4)

                    corner = (x + 5, y + 15)

                    cv2.putText(
                        frame,
                        text,
                        corner,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale,
                        fontColor,
                        thickness,
                        lineType,
                    )
                
                if len(found_this_frame) > 0:
                    all_found = list(set(all_found))
                    all_found.sort()

                    print([x[0] for x in found_this_frame])
                    print(all_found)

            unscale = 1 / HAZMAT_FRAME_SCALE
            state_hazmat["hazmat_frame"] = cv2.resize(frame, (0, 0), fx=unscale, fy=unscale)

        fps_controller.update()
        state_hazmat["hazmat_fps"] = fps_controller.fps()

        all_found = list(set(all_found))
        all_found.sort()
        state_hazmat["hazmats_found"] = all_found

        hazmat_queue.put_nowait(state_hazmat)
        # print("b")
#------------------------------------------------------------------------------#


#------------------------------------------------------------------------------#
def key_down(key):
    global SERVER_STATE
    try:
        return SERVER_STATE[key] == "true"
    except KeyError:
        return False

def main(main_queue, hazmat_queue, debug, video_capture_zero, caps):
    global SERVER_STATE, MAIN_STATE

    print("Starting cameras...")

    if video_capture_zero:
        caps["webcam1"] = cv2.VideoCapture(0)
    else:
        for key, value in cap_args.items():
            print(f'Opening camera {key} with args "{value}"...')
            caps[key] = cv2.VideoCapture(value, cv2.CAP_GSTREAMER)
            print(f"Camera {key} opened.")

    for key, cap in caps.items():
        if not cap.isOpened():
            raise RuntimeError(
                f"Can't open camera {key}. Are the cap_args set right? Is the camera plugged in?"
            )
        print(f"Camera {key} opened.")


    time.sleep(CAMERA_WAKEUP_TIME)


    print(f"\nPress '{HAZMAT_TOGGLE_KEY}' to toggle running hazmat detection.")
    print(f"Press '{HAZMAT_CLEAR_KEY}' to clear all found hazmat labels.")
    print(f"Press '{QR_TOGGLE_KEY}' to toggle running QR detection.")
    print(f"Press '{QR_CLEAR_KEY}' to clear all found QR codes.\n")
    print("Press 1-4 to switched focused feed (0 to show grid).")
    print("Press 5 to toggle sidebar.")

    fps_controller = util.FPS()

    run_hazmat_toggler = util.Toggler(False)
    run_hazmat_hold = False
    run_qr_toggler = util.Toggler(False)

    hazmat_tk = util.ToggleKey()
    qr_tk = util.ToggleKey()

    view_mode = util.ViewMode()

    all_qr_found = []

    state_main = START_STATE_MAIN
    state_hazmat = START_STATE_HAZMAT

    last_hazmat_update = time.time()

    while True:
        fps_controller.update()

        frames = {}
        for key, cap in caps.items():
            ret, frame = cap.read()

            if not ret or frame is None:
                print("Exiting ...")

            frames[key] = frame

        frame = frames["webcam1"]

        webcam1_shape = frames["webcam1"].shape
        if video_capture_zero:
            ir_frame = frames["webcam1"]
        else:
            ir_frame = cv2.resize(frames["ir"], (webcam1_shape[1], webcam1_shape[0]))

        while True:
            try:
                state_hazmat = hazmat_queue.get_nowait()
                last_hazmat_update = time.time()
            except queue.Empty:
                break

        if state_hazmat["hazmat_frame"] is not None:
            hazmat_frame = state_hazmat["hazmat_frame"]
        else:
            hazmat_frame = np.zeros_like(frame)

        frame_to_pass_to_hazmat = frame.copy()


        state_main["run_hazmat"] = run_hazmat_toggler.get() or run_hazmat_hold

        fps = fps_controller.fps()
        hazmat_fps = state_hazmat["hazmat_fps"]

        if debug:
            print(f"FPS: {fps:.0f}\tHazmat FPS: {hazmat_fps:.0f}\tHazmat: {state_main['run_hazmat']}\tQR: {run_qr_toggler}")

        if run_qr_toggler:
            start = time.time()

            qr_found_this_frame = qr_detect.qr_detect(frame)
            if len(qr_found_this_frame) > 0:
                previous_qr_count = len(all_qr_found)
                for qr in qr_found_this_frame:
                    all_qr_found.append(qr.strip())

                all_qr_found = list(set(all_qr_found))
                all_qr_found.sort()

                if len(all_qr_found) > previous_qr_count:
                    print(qr_found_this_frame)
                    print(all_qr_found)

            end = time.time()

            ratio = min((end - start) / QR_TIME_BAR_SCALE, 1)
            w = ratio * (frame.shape[1] - 10)

            cv2.line(
                frame,
                (5, 5),
                (5 + int(w), 5),
                (0, 0, 255),
                3
            )
        else:
            cv2.line(
                frame,
                (5, 5),
                (5, 5),
                (255, 255, 0),
                3
            )

        all_qr_found = list(set(all_qr_found))
        all_qr_found.sort()


        # fps text (bottom left)
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (5, frame.shape[0] - 5)
        fontScale              = 0.5
        fontColor              = (0, 255, 0)
        thickness              = 1
        lineType               = 2

        text                   = "FPS: %.0f" % fps
        cv2.putText(frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

        text                  = "Hazmat FPS: %.0f" % hazmat_fps
        cv2.putText(hazmat_frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

        time_since_last_hazmat_update = time.time() - last_hazmat_update
        ratio = min(time_since_last_hazmat_update / HAZMAT_DELAY_BAR_SCALE, 1)
        w = ratio * (frame.shape[1] - 10)

        cv2.line(
            hazmat_frame,
            (5, 5),
            (5 + int(w), 5),
            (255, 255, 0) if not state_main["run_hazmat"] else (0, 0, 255),
            3
        )

        read_state()

        if hazmat_tk.down(key_down(HAZMAT_TOGGLE_KEY)):
            run_hazmat_toggler.toggle()
        if qr_tk.down(key_down(QR_TOGGLE_KEY)):
            run_qr_toggler.toggle()

        if key_down(QR_CLEAR_KEY):
            all_qr_found = []

        if key_down(HAZMAT_CLEAR_KEY):
            state_main["clear_all_found"] = 1

        if key_down("0"):
            view_mode.mode = util.ViewMode.GRID
        elif key_down("1"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 0
        elif key_down("2"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 1
        elif key_down("3"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 2
        elif key_down("4"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 3
        
        run_hazmat_hold = key_down(HAZMAT_HOLD_KEY)

        if state_main["clear_all_found"] == 1:
            state_main["clear_all_found"] = 2
        elif state_main["clear_all_found"] == 2:
            state_main["clear_all_found"] = 0

        state_main["frame"] = frame_to_pass_to_hazmat
        main_queue.put_nowait(state_main)


        if view_mode.mode == util.ViewMode.GRID:
            top_combined = cv2.hconcat([frame, hazmat_frame])
            if video_capture_zero:
                bottom_combined = cv2.hconcat([frames["webcam1"], ir_frame])
            else:
                bottom_combined = cv2.hconcat([frames["webcam2"], ir_frame])
        else:
            if video_capture_zero:
                all_frames = [frame, hazmat_frame, frames["webcam1"], ir_frame]
            else:
                all_frames = [frame, hazmat_frame, frames["webcam2"], ir_frame]
            
            top_frames = []
            for i, f in enumerate(all_frames):
                if i != view_mode.zoom_on:
                    top_frames.append(f)

            top_combined = cv2.hconcat(top_frames)
            resize_factor = all_frames[view_mode.zoom_on].shape[1] / top_combined.shape[1]
            top_combined = cv2.resize(top_combined, (0, 0), fx=resize_factor, fy=resize_factor)
            bottom_combined = all_frames[view_mode.zoom_on]


        combined = cv2.vconcat([top_combined, bottom_combined])

        combine_downscaled = cv2.resize(combined, (0, 0), fx=SERVER_FRAME_SCALE, fy=SERVER_FRAME_SCALE)
        MAIN_STATE["frame"] = base64.b64encode(cv2.imencode('.jpg', combine_downscaled)[1]).decode()

        MAIN_STATE["w"] = combine_downscaled.shape[1]
        MAIN_STATE["h"] = combine_downscaled.shape[0]

        hazmats_found = state_hazmat["hazmats_found"]
        hazmats_found = list(set(hazmats_found))
        hazmats_found.sort()

        MAIN_STATE["hazmats_found"] = hazmats_found
        MAIN_STATE["qr_found"] = all_qr_found

        write_state()
#------------------------------------------------------------------------------#


#------------------------------------------------------------------------------#
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--debug", required=False, help="show debug prints", action="store_true")
ap.add_argument("-z", "--video-capture-zero", required=False, help="use VideoCapture(0)", action="store_true")
args = vars(ap.parse_args())
#------------------------------------------------------------------------------#


#------------------------------------------------------------------------------#
if __name__ == "__main__":
    print("Starting hazmat thread...")

    main_queue = Queue()
    hazmat_queue = Queue()

    hazmat_thread = Process(target=hazmat_main, args=(main_queue, hazmat_queue))

    hazmat_thread.start()

    print("Starting main thread...")
    
    caps = {}
    main(main_queue, hazmat_queue, args["debug"], args["video_capture_zero"], caps)

    for cap in caps.values():
        cap.release()
    cv2.destroyAllWindows()

    print("Exiting cameras...")

    START_STATE_MAIN["quit"] = True
    main_queue.put_nowait(START_STATE_MAIN)
    
    hazmat_thread.terminate()
    time.sleep(1) # wait for thread to terminate
    hazmat_thread.close()

    print("Closing queues...")

    main_queue.close()
    hazmat_queue.close()

    # basically `allow_exit_without_flush`
    main_queue.cancel_join_thread()
    hazmat_queue.cancel_join_thread()

    print("Cameras done.")
