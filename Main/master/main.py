import time
import base64

import cv2
import numpy as np
import psutil

import shared_util

import master.consts
import master.util
import master.qr_detect
import master.motion_detect

import hazmat.consts
import server.consts
import camera.consts


def thread(hazmat_dq, server_dq, camera_dqs, video_capture_zero, gpu_log_file):
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

    fps_controller = shared_util.FPSController()

    all_qr_found = []
    last_clear_qr = server.consts.STATE_FROM_SELF["clear"]["qr"]

    average_frame = None
    update_average_frame = False

    hazmat_ds = shared_util.DoubleState(hazmat.consts.STATE_FROM_MASTER, hazmat.consts.STATE_FROM_SELF)
    server_ds = shared_util.DoubleState(server.consts.STATE_FROM_MASTER, server.consts.STATE_FROM_SELF)

    camera_dses = {}
    for key in camera_dqs.keys():
        camera_ds = shared_util.DoubleState(camera.consts.STATE_FROM_MASTER, camera.consts.STATE_FROM_SELF)
        camera_dses[key] = camera_ds

    base_key = None if video_capture_zero else "webcam1"
    frame_copy = None

    killer = shared_util.GracefulKiller()

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
                frames[key] = np.zeros((camera.consts.CAMERA_SIZE[1], camera.consts.CAMERA_SIZE[0], 3), dtype=np.uint8)
                frames[key] += camera.consts.CAMERA_NONE_GREY

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
            master.util.draw_fps_text(ir_frame, camera_dses["ir"].s2["fps"])

            master.util.draw_fps_text(frames["webcam2"], camera_dses["webcam2"].s2["fps"])
        # -------------------------------------------------------------------- #
        

        # -------------------------------------------------------------------- #
        hazmat_ds.update_s2(hazmat_dq)

        if hazmat_ds.s2["hazmat_frame"] is not None:
            hazmat_frame = hazmat_ds.s2["hazmat_frame"]
        else:
            hazmat_frame = np.zeros_like(frame)

        time_since_last_hazmat_update = time.time() - hazmat_ds.s2["last_update"]
        master.util.draw_ratio_bar(
            hazmat_frame,
            time_since_last_hazmat_update / master.consts.HAZMAT_DELAY_BAR_SCALE,
            hazmat_ds.s1["run_hazmat"],
            hazmat_ds.s2["hazmat_frame"] is None
        )
        # -------------------------------------------------------------------- #


        # -------------------------------------------------------------------- #
        if server_ds.s2["run"]["qr"]:
            start = time.time()

            qr_found_this_frame = master.qr_detect.qr_detect_and_draw(frame)
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

            master.util.draw_ratio_bar(frame, (end - start) / master.consts.QR_TIME_BAR_SCALE, True)
        elif server_ds.s2["run"]["md"]:
            start = time.time()
            motion_min_area = server_ds.s2["motion_min_area"]
            motion_threshold = server_ds.s2["motion_threshold"]
            master.motion_detect.motion_detect_and_draw(frame_copy, average_frame, frame, motion_min_area, motion_threshold)
            end = time.time()

            master.util.draw_ratio_bar(frame, (end - start) / master.consts.MOTION_TIME_BAR_SCALE, True, True)
        else:
            master.util.draw_ratio_bar(frame, 0, False)

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
        master.util.draw_fps_text(frame, camera_dses[base_key].s2["fps"])
        master.util.draw_fps_text(hazmat_frame, hazmat_ds.s2["hazmat_fps"])

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
        combine_downscaled = cv2.resize(combined, (0, 0), fx=master.consts.SERVER_FRAME_SCALE, fy=master.consts.SERVER_FRAME_SCALE)
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
                -1,
            ]
        server_ds.s1["fpses"] = fpses

        server_ds.s1["ram"] = psutil.virtual_memory().percent
        server_ds.s1["cpu"] = psutil.cpu_percent()


        if not video_capture_zero:
            last_line = master.util.read_last_line(gpu_log_file)
            peices = last_line.split()
            for i, peice in enumerate(peices):
                if peice == "GR3D_FREQ":
                    section = peices[i + 1]
                    server_ds.s1["gpu"] = float(section.split("%")[0])    
                    break        

        server_ds.put_s1(server_dq)
        # -------------------------------------------------------------------- #