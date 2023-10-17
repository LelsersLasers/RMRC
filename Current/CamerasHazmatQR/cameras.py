# Runs 2 webcams, IR, QR detection, and hazmat detection.

CAP_ARGS = {
    "webcam1": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "webcam2": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "ir": "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! appsink",
}

"""
TODO:
ws vs not
"""


import cv2
import time
import numpy as np
import argparse
from multiprocessing import Process, Pool
import argparse
import base64
import queue
import util
import hazmat
import qr_detect

from flask import Flask, render_template, jsonify
import logging


HAZMAT_TOGGLE_KEY = "h"
HAZMAT_HOLD_KEY = "g"
QR_TOGGLE_KEY = "r"
HAZMAT_CLEAR_KEY = "c"
QR_CLEAR_KEY = "x"

HAZMAT_DRY_FPS = 20
CAMERA_WAKEUP_TIME = 0.5
HAZMAT_FRAME_SCALE = 1
HAZMAT_DELAY_BAR_SCALE = 5  # in seconds
QR_TIME_BAR_SCALE = 0.1  # in seconds
SERVER_FRAME_SCALE = 1
HAMZAT_POOL_SIZE = 4

# ---------------------------------------------------------------------------- #
# What main thread sends
STATE_HAZMAT_MAIN = {
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
    "last_update": 0,
}
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
STATE_CAMERA_MAIN = {
    "quit": False,
}

STATE_CAMERA = {
    "frame": None,
    "ns": 0,
    "fps": 20,
}
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
STATE_SERVER_MAIN = {
    "frame": "",
    "ns": 0,
    "w": 1,
    "h": 1,
    "hazmats_found": [],
    "qr_found": [],
    "fpses": [-1, -1, -1, -1, -1],
}
STATE_SERVER = {} # keys


app = Flask(__name__)
server_dq = util.DoubleQueue()
server_ds = util.DoubleState(STATE_SERVER_MAIN, STATE_SERVER)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/set/<key>/<value>", methods=["GET"])
def set_key(key, value):
    global server_dq, server_ds

    server_ds.s2[key] = value

    server_ds.put_s2(server_dq)

    response = jsonify(server_ds.s2)
    response.headers.add("Access-Control-Allow-Origin", "*")

    return response


@app.route("/get", methods=["GET"])
def get():
    global server_dq, server_ds
    server_ds.update_s1(server_dq)

    response = jsonify(server_ds.s1)
    response.headers.add("Access-Control-Allow-Origin", "*")

    return response
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def hazmat_main(hazmat_dq):
    time.sleep(CAMERA_WAKEUP_TIME)

    fps_controller = util.FPSController()

    all_found = []
    frame = None

    hazmat_ds = util.DoubleState(STATE_HAZMAT_MAIN, STATE_HAZMAT)

    try:
        while not hazmat_ds.s1["quit"]:

            # ---------------------------------------------------------------- #
            clear_all_found = False
            while True:
                try:
                    hazmat_ds.s1 = hazmat_dq.q1.get_nowait()
                    clear_all_found = clear_all_found or hazmat_ds.s1["clear_all_found"] > 0
                except queue.Empty:
                    break

            if clear_all_found:
                all_found = []
                print("Cleared all found hazmat labels.")
            # ---------------------------------------------------------------- #

            if hazmat_ds.s1["frame"] is not None:
                frame = hazmat_ds.s1["frame"]
                frame = cv2.resize(frame, (0, 0), fx=HAZMAT_FRAME_SCALE, fy=HAZMAT_FRAME_SCALE)

                if hazmat_ds.s1["run_hazmat"]:
                    with Pool(HAMZAT_POOL_SIZE) as pool:
                        threshVals = [90, 100, 110, 120, 130, 140, 150, 160, 170]

                        args = [(frame, threshVal) for threshVal in threshVals]
                        all_received_tups = pool.starmap(hazmat.processScreenshot, args)

                        found_this_frame = []

                        for received_tups in all_received_tups:
                            for r in received_tups:
                                text = r[0].strip()
                                cnt = r[1]
                                rect = util.Rect(cv2.boundingRect(cnt))
                                found_this_frame.append((text, cnt, rect))
                                all_found.append(text)

                    found_this_frame = util.remove_dups(found_this_frame, lambda x: x[2])

                    fontScale = 0.5
                    fontColor = (0, 0, 255)
                    thickness = 1
                    lineType = 2

                    for found in found_this_frame:
                        text, cnt, rect = found

                        frame = cv2.drawContours(frame, [cnt], -1, (255, 0, 0), 3)

                        cv2.rectangle(frame, (rect.x, rect.y), (rect.x + rect.w, rect.y + rect.h), (0, 225, 0), 4)

                        corner = (rect.x + 5, rect.y + 15)

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
                hazmat_ds.s2["hazmat_frame"] = cv2.resize(frame, (0, 0), fx=unscale, fy=unscale)

            fps_controller.update()
            hazmat_ds.s2["hazmat_fps"] = fps_controller.fps()

            all_found = list(set(all_found))
            all_found.sort()
            hazmat_ds.s2["hazmats_found"] = all_found

            hazmat_ds.s2["last_update"] = time.time()

            hazmat_ds.put_s2(hazmat_dq)

            if not hazmat_ds.s1["run_hazmat"]:
                time.sleep(1 / HAZMAT_DRY_FPS)
    except KeyboardInterrupt:
        pass
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def camera_main(camera_dq, key):
    camera_ds = util.DoubleState(STATE_CAMERA_MAIN, STATE_CAMERA)

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

            camera_ds.s2["ns"] = time.time_ns()

            if not ret or frame is None:
                print("Exiting ...")

            camera_ds.s2["frame"] = frame

            fps_controller.update()
            camera_ds.s2["fps"] = fps_controller.fps()

            camera_ds.put_s2(camera_dq)
    finally:
        print(f"Releasing camera {key}...")
        cap.release()
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def key_down(keys, key):
    try:
        return keys[key] == "true"
    except KeyError:
        return False
    
def fps_text(frame, fps):
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (5, frame.shape[0] - 5)
    fontScale = 0.5
    fontColor = (0, 255, 0)
    thickness = 1
    lineType = 2

    text = "FPS: %.0f" % fps
    cv2.putText(frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)


def main(hazmat_dq, server_dq, camera_dqs, video_capture_zero):
    print(f"\nPress '{HAZMAT_TOGGLE_KEY}' to toggle running hazmat detection.")
    print(f"Press '{HAZMAT_CLEAR_KEY}' to clear all found hazmat labels.")
    print(f"Press '{QR_TOGGLE_KEY}' to toggle running QR detection.")
    print(f"Press '{QR_CLEAR_KEY}' to clear all found QR codes.")
    print("Press 1-4 to switched focused feed (0 to show grid).")
    print("Press 5 to toggle sidebar.\n")

    fps_controller = util.FPSController()

    run_hazmat_toggler = util.Toggler(False)
    run_hazmat_hold = False
    run_qr_toggler = util.Toggler(False)

    hazmat_tk = util.ToggleKey()
    qr_tk = util.ToggleKey()

    view_mode = util.ViewMode()

    all_qr_found = []

    hazmat_ds = util.DoubleState(STATE_HAZMAT_MAIN, STATE_HAZMAT)
    server_ds = util.DoubleState(STATE_SERVER_MAIN, STATE_SERVER)

    camera_dses = {}
    for key in camera_dqs.keys():
        camera_ds = util.DoubleState(STATE_CAMERA_MAIN, STATE_CAMERA)
        camera_dses[key] = camera_ds

    base_key = None if video_capture_zero else "webcam1"

    killer = util.GracefulKiller()

    while not killer.kill_now and not hazmat_ds.s1["quit"]:
        fps_controller.update()

        # -------------------------------------------------------------------- #
        frames = {}
        frame_read_time_ns = time.time_ns()
        for key, camera_dq in camera_dqs.items():
            camera_ds = camera_dses[key]
            camera_ds.update_s2(camera_dq)
            frames[key] = camera_ds.s2["frame"]

            if key == base_key:
                frame_read_time_ns = camera_ds.s2["ns"]

        frame = frames[base_key]
        if frame is None:
            time.sleep(0.5)
            continue

        base_frame_shape = frames[base_key].shape
        if video_capture_zero:
            ir_frame = frames[base_key]
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

        frame_to_pass_to_hazmat = frame.copy()

        time_since_last_hazmat_update = time.time() - hazmat_ds.s2["last_update"]
        ratio = min(time_since_last_hazmat_update / HAZMAT_DELAY_BAR_SCALE, 1)
        w = ratio * (frame.shape[1] - 10)

        cv2.line(
            hazmat_frame, (5, 5), (5 + int(w), 5), (255, 255, 0) if not hazmat_ds.s1["run_hazmat"] else (0, 0, 255), 3
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

            ratio = min((end - start) / QR_TIME_BAR_SCALE, 1)
            w = ratio * (frame.shape[1] - 10)

            cv2.line(frame, (5, 5), (5 + int(w), 5), (0, 0, 255), 3)
        else:
            cv2.line(frame, (5, 5), (5, 5), (255, 255, 0), 3)

        all_qr_found = list(set(all_qr_found))
        all_qr_found.sort()
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        server_ds.update_s2(server_dq)

        if hazmat_tk.down(key_down(server_ds.s2, HAZMAT_TOGGLE_KEY)):
            run_hazmat_toggler.toggle()
        run_hazmat_hold = key_down(server_ds.s2, HAZMAT_HOLD_KEY)
        
        if qr_tk.down(key_down(server_ds.s2, QR_TOGGLE_KEY)):
            run_qr_toggler.toggle()

        if key_down(server_ds.s2, QR_CLEAR_KEY):
            all_qr_found = []

        if key_down(server_ds.s2, HAZMAT_CLEAR_KEY):
            hazmat_ds.s1["clear_all_found"] = 1

        if key_down(server_ds.s2, "0"):
            view_mode.mode = util.ViewMode.GRID
        elif key_down(server_ds.s2, "1"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 0
        elif key_down(server_ds.s2, "2"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 1
        elif key_down(server_ds.s2, "3"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 2
        elif key_down(server_ds.s2, "4"):
            view_mode.mode = util.ViewMode.ZOOM
            view_mode.zoom_on = 3        
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        hazmat_ds.s1["run_hazmat"] = run_hazmat_toggler.get() or run_hazmat_hold

        if hazmat_ds.s1["clear_all_found"] == 1:
            hazmat_ds.s1["clear_all_found"] = 2
        elif hazmat_ds.s1["clear_all_found"] == 2:
            hazmat_ds.s1["clear_all_found"] = 0

        hazmat_ds.s1["frame"] = frame_to_pass_to_hazmat
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
        server_ds.s1["qr_found"] = all_qr_found

        server_ds.s1["ns"] = frame_read_time_ns

        if video_capture_zero:
            # webcam1, webcam2, ir
            fpses = [camera_dses[base_key].s2["fps"]] * 3
        else:
            fpses = []
            for key in camera_dses.keys():
                fpses.append(camera_dses[key].s2["fps"])
        fpses.append(hazmat_ds.s2["hazmat_fps"]) # hazmat
        fpses.append(fps_controller.fps()) # main/master
        server_ds.s1["fpses"] = fpses

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
    # ------------------------------------------------------------------------ #
    print("\nStarting camera threads...")

    camera_dqs = {}
    camera_threads = {}

    cap_arg_keys = [None] if args["video_capture_zero"] else CAP_ARGS.keys()
    for key in cap_arg_keys:
        camera_dq = util.DoubleQueue()

        camera_thread = Process(target=camera_main, args=(camera_dq, key))
        camera_thread.start()
        print(f"Camera {key} thread pid: {camera_thread.pid}")

        camera_dqs[key] = camera_dq
        camera_threads[key] = camera_thread

    time.sleep(CAMERA_WAKEUP_TIME * 2)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting hazmat thread...")

    hazmat_dq = util.DoubleQueue()

    hazmat_thread = Process(target=hazmat_main, args=(hazmat_dq,))
    hazmat_thread.start()
    print(f"Hazmat thread pid: {hazmat_thread.pid}")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting server...")

    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    flask_thread = Process(target=app.run, kwargs={"debug": False, "port": 5000, "host": "0.0.0.0"})
    flask_thread.start()
    print(f"Flask thread pid: {flask_thread.pid}")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting main thread...\n")

    try:
        main(hazmat_dq, server_dq, camera_dqs, args["video_capture_zero"])
    except Exception as e:
        # print(e)
        pass
    # ------------------------------------------------------------------------ #


    print("\n\nExiting...")


    # ------------------------------------------------------------------------ #
    print("Closing camera threads...")
    for key in camera_threads.keys():
        print(f"Closing camera {key} capture and thread...")
        STATE_CAMERA_MAIN["quit"] = True

        camera_dq = camera_dqs[key]
        camera_dq.put_q1(STATE_CAMERA_MAIN)

        camera_thread = camera_threads[key]
        util.close_thread(camera_thread)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing hazmat thread...")
    STATE_HAZMAT_MAIN["quit"] = True
    hazmat_dq.put_q1(STATE_HAZMAT_MAIN)

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
