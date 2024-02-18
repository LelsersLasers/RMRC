# CAP_ARGS = {
#     "webcam1": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videorate ! video/x-raw,framerate=30/1 ! videoconvert ! appsink",
#     "webcam2": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videorate ! video/x-raw,framerate=30/1 ! videoconvert ! appsink",
#     "ir": "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! appsink",
# }

CAP_ARGS = {
    "webcam1": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "webcam2": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "ir": "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! appsink",
}
CAMERA_SIZE = (320, 240)


import time
import argparse
import base64
import traceback
from multiprocessing import Process

import util
import qr_detect
import motion_detect

import hazmat.consts
import hazmat.main

import motors.consts
import motors.main

import server.consts
import server.main

import cv2
import numpy as np
import psutil


GPU_LOG_FILENAME = "tegrastats.log"
SERVER_FRAME_SCALE = 1

HAZMAT_DELAY_BAR_SCALE = 2  # in seconds
QR_TIME_BAR_SCALE = 0.1     # in seconds
MOTION_TIME_BAR_SCALE = 0.1 # in seconds

CAMERA_WAKEUP_TIME = 1.5
CAMERA_NONE_GREY = 50


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

            camera_ds.s2["frame"] = frame

            fps_controller.update()
            camera_ds.s2["fps"] = fps_controller.fps()

            camera_ds.put_s2(camera_dq)
    except KeyboardInterrupt: pass
    finally:
        print(f"Releasing camera {key}...")
        cap.release()
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
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


def master_main(hazmat_dq, server_dq, camera_dqs, video_capture_zero, gpu_log_file):
    print("\nHold 'wasd' to move the robot.")
    print("Hold 'z' to set all motor speeds to 0.")
    print("Press 'h' to toggle running hazmat detection.")
    print("Press 'r' to toggle running qr detection.")
    print("Press 'm' to toggle running motion detection.")
    print("Press 'c' to clear all found hazmat labels.")
    print("Press 'x' to clear all found QR codes.")
    print("Press 't'/'T' to increase/decrease power by 20%.")
    print("Press 1-4 to switched focused feed (0 to show grid).")
    print("Press 5 to toggle sidebar.\n")

    fps_controller = util.FPSController()

    all_qr_found = []
    last_clear_qr = server.consts.STATE_FROM_SELF["clear"]["qr"]

    average_frame = None
    update_average_frame = False

    hazmat_ds = util.DoubleState(hazmat.consts.STATE_FROM_MASTER, hazmat.consts.STATE_FROM_SELF)
    server_ds = util.DoubleState(server.consts.STATE_FROM_MASTER, server.consts.STATE_FROM_SELF)

    camera_dses = {}
    for key in camera_dqs.keys():
        camera_ds = util.DoubleState(STATE_CAMERA_MASTER, STATE_CAMERA)
        camera_dses[key] = camera_ds

    base_key = None if video_capture_zero else "webcam1"
    frame_copy = None

    killer = util.GracefulKiller()

    frame_read_time = time.time()

    while not killer.kill_now and not hazmat_ds.s1["quit"]:
        fps_controller.update()

        # -------------------------------------------------------------------- #
        frames = {}
        for key, camera_dq in camera_dqs.items():
            camera_ds = camera_dses[key]
            camera_ds.update_s2(camera_dq)
            frames[key] = camera_ds.s2["frame"]

            if frames[key] is None:
                frames[key] = np.zeros((CAMERA_SIZE[1], CAMERA_SIZE[0], 3), dtype=np.uint8) + CAMERA_NONE_GREY

            if key == base_key and frames[key] is not None and camera_ds.s2["time"] > frame_read_time:
                frame_read_time = camera_ds.s2["time"]

                frame_copy = frames[key].copy()

                if average_frame is None:
                    average_frame = frames[key].copy().astype("float")
                update_average_frame = True

        frame = frames[base_key]            

        base_frame_shape = frame.shape
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

        time_since_last_hazmat_update = time.time() - hazmat_ds.s2["last_update"]
        ratio_bar(
            hazmat_frame,
            time_since_last_hazmat_update / HAZMAT_DELAY_BAR_SCALE,
            hazmat_ds.s1["run_hazmat"],
            hazmat_ds.s2["hazmat_frame"] is None
        )
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        if server_ds.s2["run"]["qr"]:
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
        elif server_ds.s2["run"]["md"]:
            start = time.time()
            motion_min_area = server_ds.s2["motion_min_area"]
            motion_threshold = server_ds.s2["motion_threshold"]
            motion_detect.motion_detect_and_draw(frame_copy, average_frame, frame, motion_min_area, motion_threshold)
            end = time.time()

            ratio_bar(frame, (end - start) / MOTION_TIME_BAR_SCALE, True, True)
        else:
            ratio_bar(frame, 0, False)

        if update_average_frame:
            update_average_frame = False
            motion_new_frame_weight = server_ds.s2["motion_new_frame_weight"]
            cv2.accumulateWeighted(frame_copy.astype("float"), average_frame, motion_new_frame_weight)
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        server_ds.update_s2(server_dq)

        if server_ds.s2["clear"]["qr"] > last_clear_qr:
            last_clear_qr = server_ds.s2["clear"]["qr"]
            all_qr_found = []
        # -------------------------------------------------------------------- #
            

        # -------------------------------------------------------------------- #
        hazmat_ds.s1["run_hazmat"] = server_ds.s2["run"]["hazmat"]
        hazmat_ds.s1["clear"] = server_ds.s2["clear"]["hazmat"]

        hazmat_ds.s1["hazmat_levenshtein_thresh"] = server_ds.s2["hazmat_levenshtein_thresh"]
        hazmat_ds.s1["hazmat_angle_change"] = server_ds.s2["hazmat_angle_change"]

        hazmat_ds.s1["frame"] = frame_copy
        hazmat_ds.put_s1(hazmat_dq)
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        fps_text(frame, camera_dses[base_key].s2["fps"])
        fps_text(hazmat_frame, hazmat_ds.s2["hazmat_fps"])

        if server_ds.s2["view_mode"] == 0:
            top_combined = cv2.hconcat([frame, hazmat_frame])
            if video_capture_zero:
                bottom_combined = cv2.hconcat([frames[base_key], ir_frame])
            else:
                bottom_combined = cv2.hconcat([frames["webcam2"], ir_frame])
        else:
            zoom_on = server_ds.s2["view_mode"] - 1
            if video_capture_zero:
                all_frames = [frame, hazmat_frame, frames[base_key], ir_frame]
            else:
                all_frames = [frame, hazmat_frame, frames["webcam2"], ir_frame]

            top_frames = []
            for i, f in enumerate(all_frames):
                if i != zoom_on:
                    top_frames.append(f)

            top_combined = cv2.hconcat(top_frames)
            resize_factor = all_frames[zoom_on].shape[1] / top_combined.shape[1]
            top_combined = cv2.resize(top_combined, (0, 0), fx=resize_factor, fy=resize_factor)
            bottom_combined = all_frames[zoom_on]

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
                -1,
            ]
        else:
            fpses = [
                camera_dses["webcam1"].s2["fps"],
                hazmat_ds.s2["hazmat_fps"],
                camera_dses["webcam2"].s2["fps"],
                camera_dses["ir"].s2["fps"],
                fps_controller.fps(),
                -1,
            ]
        server_ds.s1["fpses"] = fpses

        server_ds.s1["ram"] = psutil.virtual_memory().percent
        server_ds.s1["cpu"] = psutil.cpu_percent()

        if not zero_video_capture:
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
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-z", "--video-capture-zero", required=False, help="use VideoCapture(0)", action="store_true")
    args = vars(ap.parse_args())

    zero_video_capture = args["video_capture_zero"]
    # ------------------------------------------------------------------------ #

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

    hazmat_thread = Process(target=hazmat.main.thread, args=(hazmat_dq,))
    hazmat_thread.daemon = True
    hazmat_thread.start()
    print(f"Hazmat thread pid: {hazmat_thread.pid}")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting server...")

    server_dq = util.DoubleQueue()
    server_motor_dq = util.DoubleQueue()

    flask_thread = Process(target=server.main.thread, args=(server_dq, server_motor_dq))
    flask_thread.daemon = True
    flask_thread.start()
    print(f"Flask thread pid: {flask_thread.pid}")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting motor thread...")

    motor_dq = util.DoubleQueue()
    motor_ds = util.DoubleState(motors.consts.STATE_FROM_MASTER, {})

    motor_thread = Process(target=motors.main.thread, args=(server_motor_dq, motor_dq, zero_video_capture))
    motor_thread.daemon = True
    motor_thread.start()
    print(f"Motor thread pid: {motor_thread.pid}")
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("\nStarting master thread...\n")

    try:
        gpu_log_file = None if zero_video_capture else open(GPU_LOG_FILENAME, 'rb')
        master_main(hazmat_dq, server_dq, camera_dqs, zero_video_capture, gpu_log_file)
    except Exception as e:
        print("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRROOOOOOOOOOOORRRRRRRRR", e)
        print(traceback.format_exc())
    # except: pass
    finally:
        if not zero_video_capture:
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
    hazmat.consts.STATE_FROM_MASTER["quit"] = True
    hazmat_dq.put_q1(hazmat.consts.STATE_FROM_MASTER)

    util.close_thread(hazmat_thread)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing server...")
    util.close_thread(flask_thread)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing motor thread...")
    motors.consts.STATE_FROM_MASTER["quit"] = True
    motor_dq.put_q1(motors.consts.STATE_FROM_MASTER)

    util.close_thread(motor_thread)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    print("Closing queues...")

    hazmat_dq.close()
    server_dq.close()
    server_motor_dq.close()

    for camera_dq in camera_dqs.values():
        camera_dq.close()
    # ------------------------------------------------------------------------ #

    print("Done.")
