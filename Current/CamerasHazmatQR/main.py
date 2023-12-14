# Runs 2 webcams, IR, QR detection, motion detection, and hazmat detection.

CAP_ARGS = {
    "webcam1": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videorate ! video/x-raw,framerate=30/1 ! videoconvert ! appsink",
    "webcam2": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videorate ! video/x-raw,framerate=30/1 ! videoconvert ! appsink",
    "ir": "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! appsink",
}


import time
import argparse
import base64
import traceback
from multiprocessing import Process

import util
import hazmat
import qr_detect
import motion_detect
import motors

import cv2
import numpy as np
import psutil
import easyocr

from flask import Flask, render_template, jsonify
import logging


HAZMAT_TOGGLE_KEY = "h"
HAZMAT_HOLD_KEY = "g"
QR_TOGGLE_KEY = "r"
MOTION_TOGGLE_KEY = "m"
HAZMAT_CLEAR_KEY = "c"
QR_CLEAR_KEY = "x"

GPU_LOG_FILENAME = "tegrastats.log"
HAZMAT_LEVENSHTEIN_THRESH = 0.4
HAZMAT_DRY_FPS = 15
CAMERA_WAKEUP_TIME = 1.0
HAZMAT_ANGLE = 90
HAZMAT_DELAY_BAR_SCALE = 2  # in seconds
QR_TIME_BAR_SCALE = 0.1     # in seconds
MOTION_TIME_BAR_SCALE = 0.1 # in seconds
MOTION_MIN_AREA = 500
MOTION_THRESHOLD = 65
MOTION_NEW_FRAME_WEIGHT = 0.4
SERVER_FRAME_SCALE = 1

# TODO
MAX_SPEED = 330
DIAGONAL_MULTIPLIER = 0.4

# ---------------------------------------------------------------------------- #
# What master thread sends
STATE_HAZMAT_MASTER = {
    "frame": None,
    "run_hazmat": False,
    "quit": False,
    "clear_all_found": 0,
}

# What hazmat thread sends
STATE_HAZMAT = {
    "hazmat_fps": HAZMAT_DRY_FPS,
    "hazmat_frame": None,
    "hazmats_found": [],
    "last_update": time.time(),
    "angle": 0,
}
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
STATE_CAMERA_MASTER = {
    "quit": False,
}

STATE_CAMERA = {
    "frame": None,
    "ns": 0,
    "fps": 20,
    "time": time.time(),
}
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
STATE_SERVER_MASTER = {
    "frame": "",
    "ns": 0,
    "w": 1,
    "h": 1,
    "hazmats_found": [],
    "qr_found": [],
    "fpses": [-1, -1, -1, -1, -1],
    "ram": 0,
    "cpu": 0,
    "gpu": -1,
    "angle": 0,
    "motors": {
        "left": 0,
        "right": 0,
    }
}
STATE_SERVER = {
    "keys": [],
    "power": 100,
}
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def server_main(server_dq):
    app = Flask(__name__)
    server_ds = util.DoubleState(STATE_SERVER_MASTER, STATE_SERVER)

    @app.route("/")
    def index():
        return render_template("index.html")
    
    @app.route("/calibrate", methods=["GET"])
    def calibrate():
        now = time.time()
        response = jsonify(now)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response
    
    @app.route("/power/<value>", methods=["GET"])
    def power(value):
        server_ds.s2["power"] = float(value) / 100

        server_ds.put_s2(server_dq)

        response = jsonify(server_ds.s2)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response
    
    @app.route("/keys/", methods=["GET"])
    def no_keys_down():
        server_ds.s2["keys"] = []

        server_ds.put_s2(server_dq)

        response = jsonify(server_ds.s2)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response

    @app.route("/keys/<keys_str>", methods=["GET"])
    def keys(keys_str):
        keys = keys_str.split("-")
        server_ds.s2["keys"] = keys

        server_ds.put_s2(server_dq)

        response = jsonify(server_ds.s2)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response

    @app.route("/get", methods=["GET"])
    def get():
        server_ds.update_s1(server_dq)

        response = jsonify(server_ds.s1)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response

    app.run(debug=False, port=5000, host="0.0.0.0")
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def hazmat_main(hazmat_dq, levenshtein_thresh):
    fps_controller = util.FPSController()

    all_found = []
    frame = None

    levenshtein_results = {}

    hazmat_ds = util.DoubleState(STATE_HAZMAT_MASTER, STATE_HAZMAT)

    print("Creating easyocr reader...")
    reader = easyocr.Reader(["en"], gpu=True)
    print("easyocr reader created.")

    try:
        while not hazmat_ds.s1["quit"]:
            hazmat_ds.update_s1(hazmat_dq)

            # ---------------------------------------------------------------- #
            if hazmat_ds.s1["clear_all_found"] > 0:
                all_found = []
                print("Cleared all found hazmat labels.")
            # ---------------------------------------------------------------- #

            if hazmat_ds.s1["frame"] is not None:
                frame = hazmat_ds.s1["frame"]

                if hazmat_ds.s1["run_hazmat"] or hazmat_ds.s2["angle"] != 0:

                    frame_results = hazmat.processScreenshot(frame, hazmat_ds.s2["angle"], reader, levenshtein_thresh)
                    levenshtein_results[hazmat_ds.s2["angle"]] = frame_results

                    fontScale = 0.5
                    fontColor = (0, 0, 255)
                    thickness = 1
                    lineType = 2

                    for results in levenshtein_results.values():
                        for result in results:
                            all_found.append(result.closest)

                            frame = cv2.drawContours(frame, [result.detection_result.cnt.cnt], -1, (255, 0, 0), 3)

                            x, y, w, h = cv2.boundingRect(result.detection_result.cnt.cnt)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 225, 0), 4)

                            corner = (x + 5, y - 10)

                            cv2.putText(
                                frame,
                                result.string,
                                corner,
                                cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale,
                                fontColor,
                                thickness,
                                lineType,
                            )

                    if len(levenshtein_results[hazmat_ds.s2["angle"]]) > 0:
                        all_found = list(set(all_found))
                        all_found.sort()

                        print([result.string for result in levenshtein_results[hazmat_ds.s2["angle"]]])
                        print(all_found)

                    hazmat_ds.s2["angle"] += HAZMAT_ANGLE
                    hazmat_ds.s2["angle"] %= 360
                else:
                    hazmat_ds.s2["angle"] = 0
                    levenshtein_results = {}

                hazmat_ds.s2["hazmat_frame"] = frame

            # ---------------------------------------------------------------- #
            fps_controller.update()
            hazmat_ds.s2["hazmat_fps"] = fps_controller.fps()

            all_found = list(set(all_found))
            all_found.sort()
            hazmat_ds.s2["hazmats_found"] = all_found

            hazmat_ds.s2["last_update"] = time.time()

            hazmat_ds.put_s2(hazmat_dq)
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            if not hazmat_ds.s1["run_hazmat"]:
                time.sleep(1 / HAZMAT_DRY_FPS)
            # ---------------------------------------------------------------- #
    except KeyboardInterrupt:
        pass
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def camera_main(camera_dq, key):
    camera_ds = util.DoubleState(STATE_CAMERA_MASTER, STATE_CAMERA)

    print(f"Opening camera {key}...")
    if key is not None:
        cap = cv2.VideoCapture(CAP_ARGS[key], cv2.CAP_GSTREAMER)
    else:
        cap = cv2.VideoCapture(0)
    print(f"Camera {key} VideoCapture created.")

    if not cap.isOpened():
        raise RuntimeError(f"Can't open camera {key}. Are the cap_args set right? Is the camera plugged in?")
    print(f"Camera {key} opened.")

    time.sleep(CAMERA_WAKEUP_TIME)

    fps_controller = util.FPSController()

    try:
        while not camera_ds.s1["quit"]:
            camera_ds.update_s1(camera_dq)

            ret, frame = cap.read()
            if not ret or frame is None:
                print(f"Camera {key} read failed.")
                break

            camera_ds.s2["time"] = time.time()

            if not ret or frame is None:
                print("Exiting ...")

            camera_ds.s2["frame"] = frame

            fps_controller.update()
            camera_ds.s2["fps"] = fps_controller.fps()

            camera_ds.put_s2(camera_dq)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"Releasing camera {key}...")
        cap.release()
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def key_down(keys, key):
    return key in keys
    
def fps_text(frame, fps):
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (5, frame.shape[0] - 5)
    fontScale = 0.5
    fontColor = (0, 255, 0)
    thickness = 1
    lineType = 2

    text = "FPS: %.0f" % fps
    cv2.putText(frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

def ratio_bar(frame, ratio, active, loading = False):
    ratio = min(ratio, 1)
    w = ratio * (frame.shape[1] - 10)

    color = (0, 255, 0) if loading else ((0, 0, 255) if active else (255, 255, 0))

    cv2.line(frame, (5, 5), (5 + int(w), 5), color, 3)


def master_main(hazmat_dq, server_dq, camera_dqs, dxl_controller, video_capture_zero, gpu_log_file):
    print(f"\nPress '{HAZMAT_TOGGLE_KEY}' to toggle running hazmat detection.")
    print(f"Press '{HAZMAT_HOLD_KEY}' to run hazmat detection while holding key.")
    print(f"Press '{HAZMAT_CLEAR_KEY}' to clear all found hazmat labels.")
    print(f"Press '{QR_TOGGLE_KEY}' to toggle running QR detection.")
    print(f"Press '{QR_CLEAR_KEY}' to clear all found QR codes.")
    print(f"Press '{MOTION_TOGGLE_KEY}' to toggle running motion detection.")
    print(f"Press 't'/'T' to increase/decrease power by 20%.")
    print("Press 1-4 to switched focused feed (0 to show grid).")
    print("Press 5 to toggle sidebar.\n")

    fps_controller = util.FPSController()

    run_hazmat_toggler = util.Toggler(False)
    run_hazmat_hold = False
    run_qr_toggler = util.Toggler(False)
    run_motion_toggler = util.Toggler(False)

    hazmat_tk = util.ToggleKey()
    qr_tk = util.ToggleKey()
    motion_tk = util.ToggleKey()

    view_mode = util.ViewMode()

    if not video_capture_zero:
        dxl_controller.set_torque_status(True)

    all_qr_found = []

    average_frame = None
    update_average_frame = False

    hazmat_ds = util.DoubleState(STATE_HAZMAT_MASTER, STATE_HAZMAT)
    server_ds = util.DoubleState(STATE_SERVER_MASTER, STATE_SERVER)

    camera_dses = {}
    for key in camera_dqs.keys():
        camera_ds = util.DoubleState(STATE_CAMERA_MASTER, STATE_CAMERA)
        camera_dses[key] = camera_ds

    base_key = None if video_capture_zero else "webcam1"
    frame_copy = None

    killer = util.GracefulKiller()

    last_base_frame_time = time.time()

    while not killer.kill_now and not hazmat_ds.s1["quit"]:
        fps_controller.update()

        # -------------------------------------------------------------------- #
        frames = {}
        frame_read_time = time.time()
        for key, camera_dq in camera_dqs.items():
            camera_ds = camera_dses[key]
            camera_ds.update_s2(camera_dq)
            frames[key] = camera_ds.s2["frame"]

            if key == base_key and frames[key] is not None and camera_ds.s2["time"] > last_base_frame_time:
                frame_read_time = camera_ds.s2["time"]
                last_base_frame_time = frame_read_time

                frame_copy = frames[key].copy()

                if average_frame is None:
                    average_frame = frames[key].copy().astype("float")
                update_average_frame = True

        frame = frames[base_key]            

        if frame is None:
            time.sleep(0.5)
            continue

        base_frame_shape = frame.shape
        if video_capture_zero:
            ir_frame = frames[base_key]
        else:
            if frames["ir"] is None:
                ir_frame = np.zeros_like(frame)
            else:
                ir_frame = cv2.resize(frames["ir"], (base_frame_shape[1], base_frame_shape[0]))
            fps_text(ir_frame, camera_dses["ir"].s2["fps"])

            fps_text(frames["webcam2"], camera_dses["webcam2"].s2["fps"])
        # -------------------------------------------------------------------- #
        

        # -------------------------------------------------------------------- #
        hazmat_ds.update_s2(hazmat_dq)

        if hazmat_ds.s2["hazmat_frame"] is not None:
            hazmat_frame = hazmat_ds.s2["hazmat_frame"]
        else:
            hazmat_frame = np.zeros_like(frame)

        time_since_last_hazmat_update = time.time() - hazmat_ds.s2["last_update"]
        ratio_bar(
            hazmat_frame,
            time_since_last_hazmat_update / HAZMAT_DELAY_BAR_SCALE,
            hazmat_ds.s1["run_hazmat"],
            hazmat_ds.s2["hazmat_frame"] is None
        )
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
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

            ratio_bar(frame, (end - start) / QR_TIME_BAR_SCALE, True)
        elif run_motion_toggler:
            start = time.time()
            motion_detect.motion_detect_and_draw(frame_copy, average_frame, frame, MOTION_MIN_AREA, MOTION_THRESHOLD)
            end = time.time()

            ratio_bar(frame, (end - start) / MOTION_TIME_BAR_SCALE, True, True)
        else:
            ratio_bar(frame, 0, False)

        if update_average_frame:
            update_average_frame = False
            cv2.accumulateWeighted(frame_copy.astype("float"), average_frame, MOTION_NEW_FRAME_WEIGHT)

        all_qr_found = list(set(all_qr_found))
        all_qr_found.sort()
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        server_ds.update_s2(server_dq)

        if hazmat_tk.down(key_down(server_ds.s2["keys"], HAZMAT_TOGGLE_KEY)):
            run_hazmat_toggler.toggle()
        run_hazmat_hold = key_down(server_ds.s2["keys"], HAZMAT_HOLD_KEY)
        
        if qr_tk.down(key_down(server_ds.s2["keys"], QR_TOGGLE_KEY)):
            run_qr_toggler.toggle()
        if motion_tk.down(key_down(server_ds.s2["keys"], MOTION_TOGGLE_KEY)):
            run_motion_toggler.toggle()

        if key_down(server_ds.s2["keys"], QR_CLEAR_KEY):
            all_qr_found = []

        if key_down(server_ds.s2["keys"], HAZMAT_CLEAR_KEY):
            hazmat_ds.s1["clear_all_found"] = 1

        if key_down(server_ds.s2["keys"], "0"):
            view_mode.mode = util.ViewMode.GRID
        elif key_down(server_ds.s2["keys"], "1"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 0
        elif key_down(server_ds.s2["keys"], "2"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 1
        elif key_down(server_ds.s2["keys"], "3"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 2
        elif key_down(server_ds.s2["keys"], "4"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 3
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        if not video_capture_zero:
            # TODO
            base_speed = MAX_SPEED * server_ds.s2["power"]

            x_input = 0
            y_input = 0

            if key_down(server_ds.s2["keys"], "w"):
                y_input += 1
            if key_down(server_ds.s2["keys"], "s"):
                y_input -= -1
            if key_down(server_ds.s2["keys"], "a"):
                x_input -= 1
            if key_down(server_ds.s2["keys"], "d"):
                x_input += 1

            def match_x(x): # in terms of left side
                if x < 0:   return -1
                elif x > 0: return 1
                else:       return 0

            if y_input == 0:
                dxl_controller.speeds["left"] = match_x(x_input) * base_speed
                dxl_controller.speeds["right"] = -match_x(x_input) * base_speed
            else:
                dxl_controller.speeds["left"] = y_input * base_speed
                dxl_controller.speeds["right"] = y_input * base_speed

                if x_input < 0:
                    dxl_controller.speeds["left"] *= DIAGONAL_MULTIPLIER
                elif x_input > 0:
                    dxl_controller.speeds["right"] *= DIAGONAL_MULTIPLIER

            print(dxl_controller.speeds)

            dxl_controller.update_speed()
            dxl_controller.check_errors()
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        hazmat_ds.s1["run_hazmat"] = run_hazmat_toggler.get() or run_hazmat_hold

        if hazmat_ds.s1["clear_all_found"] == 1:
            hazmat_ds.s1["clear_all_found"] = 2
        elif hazmat_ds.s1["clear_all_found"] == 2:
            hazmat_ds.s1["clear_all_found"] = 0

        hazmat_ds.s1["frame"] = frame_copy
        hazmat_ds.put_s1(hazmat_dq)
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        fps_text(frame, camera_dses[base_key].s2["fps"])
        fps_text(hazmat_frame, hazmat_ds.s2["hazmat_fps"])

        if view_mode.mode == util.ViewMode.GRID:
            top_combined = cv2.hconcat([frame, hazmat_frame])
            if video_capture_zero:
                bottom_combined = cv2.hconcat([frames[base_key], ir_frame])
            else:
                bottom_combined = cv2.hconcat([frames["webcam2"], ir_frame])
        else:
            if video_capture_zero:
                all_frames = [frame, hazmat_frame, frames[base_key], ir_frame]
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
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        server_ds.s1["frame"] = base64.b64encode(cv2.imencode(".jpg", combine_downscaled)[1]).decode()

        server_ds.s1["w"] = combine_downscaled.shape[1]
        server_ds.s1["h"] = combine_downscaled.shape[0]

        server_ds.s1["hazmats_found"] = hazmat_ds.s2["hazmats_found"]
        server_ds.s1["angle"] = hazmat_ds.s2["angle"]
        server_ds.s1["qr_found"] = all_qr_found

        server_ds.s1["time"] = frame_read_time

        if video_capture_zero:
            fpses = [
                camera_dses[base_key].s2["fps"],
                hazmat_ds.s2["hazmat_fps"],
                camera_dses[base_key].s2["fps"],
                camera_dses[base_key].s2["fps"],
                fps_controller.fps(),
            ]
        else:
            fpses = [
                camera_dses["webcam1"].s2["fps"],
                hazmat_ds.s2["hazmat_fps"],
                camera_dses["webcam2"].s2["fps"],
                camera_dses["ir"].s2["fps"],
                fps_controller.fps(),
            ]
        server_ds.s1["fpses"] = fpses

        server_ds.s1["ram"] = psutil.virtual_memory().percent
        server_ds.s1["cpu"] = psutil.cpu_percent()

        if not zero_video_capture:
            server_ds.s1["motors"]["left"] = dxl_controller.speeds["left"] / MAX_SPEED
            server_ds.s1["motors"]["right"] = dxl_controller.speeds["right"] / MAX_SPEED

            last_line = util.read_last_line(gpu_log_file)
            peices = last_line.split()
            for i, peice in enumerate(peices):
                if peice == "GR3D_FREQ":
                    section = peices[i + 1]
                    server_ds.s1["gpu"] = float(section.split("%")[0])    
                    break        

        server_ds.put_s1(server_dq)
        # -------------------------------------------------------------------- #
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
ap = argparse.ArgumentParser()
ap.add_argument("-z", "--video-capture-zero", required=False, help="use VideoCapture(0)", action="store_true")
args = vars(ap.parse_args())
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
if __name__ == "__main__":
    zero_video_capture = args["video_capture_zero"]
    # ------------------------------------------------------------------------ #
    print("\nStarting camera threads...")

    camera_dqs = {}
    camera_threads = {}

    cap_arg_keys = [None] if zero_video_capture else CAP_ARGS.keys()
    for key in cap_arg_keys:
        camera_dq = util.DoubleQueue()

        camera_thread = Process(target=camera_main, args=(camera_dq, key))
        camera_thread.daemon = True
        camera_thread.start()
        print(f"Camera {key} thread pid: {camera_thread.pid}")

        camera_dqs[key] = camera_dq
        camera_threads[key] = camera_thread

    time.sleep(CAMERA_WAKEUP_TIME * 2)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting hazmat thread...")

    hazmat_dq = util.DoubleQueue()

    hazmat_thread = Process(
        target=hazmat_main,
        args=(hazmat_dq, HAZMAT_LEVENSHTEIN_THRESH)
    )
    hazmat_thread.daemon = True
    hazmat_thread.start()
    print(f"Hazmat thread pid: {hazmat_thread.pid}")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting server...")

    server_dq = util.DoubleQueue()

    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    flask_thread = Process(target=server_main, args=(server_dq,))
    flask_thread.daemon = True
    flask_thread.start()
    print(f"Flask thread pid: {flask_thread.pid}")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting master thread...\n")

    try:
        dxl_controller = None if zero_video_capture else motors.DynamixelController()
        gpu_log_file = None if zero_video_capture else open(GPU_LOG_FILENAME, 'rb')
        master_main(hazmat_dq, server_dq, camera_dqs, dxl_controller, zero_video_capture, gpu_log_file)
    except Exception as e:
        print("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRROOOOOOOOOOOORRRRRRRRR", e)
        print(traceback.format_exc())
    # except:
    #     pass
    finally:
        if not zero_video_capture:
            dxl_controller.set_torque_status(False)
            dxl_controller.close_port()

            gpu_log_file.close()
    # ------------------------------------------------------------------------ #


    print("\n\nExiting...")


    # ------------------------------------------------------------------------ #
    print("Closing camera threads...")
    for key in camera_threads.keys():
        print(f"Closing camera {key} capture and thread...")
        STATE_CAMERA_MASTER["quit"] = True

        camera_dq = camera_dqs[key]
        camera_dq.put_q1(STATE_CAMERA_MASTER)

        camera_thread = camera_threads[key]
        util.close_thread(camera_thread)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing hazmat thread...")
    STATE_HAZMAT_MASTER["quit"] = True
    hazmat_dq.put_q1(STATE_HAZMAT_MASTER)

    util.close_thread(hazmat_thread)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing server...")
    util.close_thread(flask_thread)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing queues...")

    hazmat_dq.close()
    server_dq.close()

    for camera_dq in camera_dqs.values():
        camera_dq.close()
    # ------------------------------------------------------------------------ #

    print("Done.")
