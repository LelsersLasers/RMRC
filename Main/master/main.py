import time
import base64

import cv2
import numpy as np
import psutil

import shared_util

import master.consts
import master.util

import detection.consts
import server.consts
import camera.consts


def thread(detection_dq, server_dq, camera_sqs, video_capture_zero):
    print("\nControls:")
    print("Hold 'wasd' to move the robot.")
    print("Hold 'z' to set all motor speeds to 0.")
    print("Press 'h' to toggle running hazmat detection.")
    print("Press 'r' to toggle running qr detection.")
    print("Press 'm' to toggle running motion detection.")
    print("Press 'c' to clear all found hazmat labels.")
    print("Press 'x' to clear all found QR codes.")
    print("Press 't'/'T' to increase/decrease power by 20%.")
    print("Press 'l' to invert controls.")
    print("Press 1-4 to switched focused feed (0 to show grid).")
    print("Press 5 to toggle sidebar.\n")

    # TODO: fix average frames including the most recent frame


    average_frame = None
    update_average_frame = False

    detection_ds = shared_util.DoubleState(detection.consts.STATE_FROM_SERVER, detection.consts.STATE_FROM_SELF)
    server_ds = shared_util.DoubleState(server.consts.STATE_FROM_MASTER, server.consts.STATE_FROM_SELF)

    gpu_log_file = None if video_capture_zero else open(master.consts.GPU_LOG_FILENAME, 'rb')

    camera_sses = {}
    frame_read_times = {}
    for key in camera_sqs.keys():
        camera_ss = shared_util.SingleState(camera.consts.STATE_FROM_SELF)
        camera_sses[key] = camera_ss
        frame_read_times[key] = 0

    last_detection_time = 0

    frame_copy = None

    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()

    try:
        while not graceful_killer.kill_now:
            fps_controller.update()

            # ---------------------------------------------------------------- #
            server_ds.update_s2(server_dq)

            base_key = None if video_capture_zero else ("webcam1" if not server_ds.s2["invert"] else "webcam2")
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            frames = {}
            update_base_frame = server_ds.s2["run"]["md"] or server_ds.s2["run"]["qr"]
            for key, camera_sq in camera_sqs.items():
                camera_ss = camera_sses[key]
                camera_ss.update_s(camera_sq)
                frames[key] = camera_ss.s["frame"]

                if frames[key] is None:
                    frames[key]  = np.zeros((camera.consts.CAMERA_SIZE[1], camera.consts.CAMERA_SIZE[0], 3), dtype=np.uint8)
                    frames[key] += camera.consts.CAMERA_NONE_GREY

                if camera_ss.s["time"] > frame_read_times[key]:
                    frame_read_times[key] = camera_ss.s["time"]

                    if key == base_key and frames[key] is not None:
                        frame_copy = frames[key].copy()

                        update_base_frame = True
                        update_average_frame = True

                        if average_frame is None:
                            average_frame = frames[key].copy().astype("float")
                    elif key != base_key:
                        server_ds.s1["frames"][key] = base64.b64encode(cv2.imencode(".jpg", frames[key])[1]).decode()

            frame = frames[base_key]            
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            detection_ds.update_s2(detection_dq)

            if detection_ds.s2["last_update"] > last_detection_time:
                last_detection_time = detection_ds.s2["last_update"]

                if detection_ds.s2["frame"] is not None:
                    detection_frame = detection_ds.s2["frame"]
                else:
                    detection_frame = np.zeros_like(frame)

                server_ds.s1["frames"]["detection"] = base64.b64encode(cv2.imencode(".jpg", detection_frame)[1]).decode()
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            detection_ds.s1["run"] = server_ds.s2["run"]
            detection_ds.s1["clear"] = server_ds.s2["clear"]

            detection_ds.s1["hazmat_levenshtein_thresh"] = server_ds.s2["hazmat_levenshtein_thresh"]
            detection_ds.s1["hazmat_angle_change"] = server_ds.s2["hazmat_angle_change"]

            detection_ds.s1["motion_min_area"] = server_ds.s2["motion_min_area"]
            detection_ds.s1["motion_threshold"] = server_ds.s2["motion_threshold"]
            detection_ds.s1["motion_new_frame_weight"] = server_ds.s2["motion_new_frame_weight"]

            detection_ds.s1["frame"] = frame_copy
            detection_ds.s1["average_frame"] = average_frame.copy()
            detection_ds.put_s1(detection_dq)
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            if update_average_frame:
                update_average_frame = False
                motion_new_frame_weight = server_ds.s2["motion_new_frame_weight"]
                cv2.accumulateWeighted(frame_copy.astype("float"), average_frame, motion_new_frame_weight)
            # ---------------------------------------------------------------- #

            # -----------------------------------------------------------------#
            if video_capture_zero:
                server_ds.s1["frames"]["webcam2"] = server_ds.s1["frames"]["webcam1"]
                server_ds.s1["frames"]["ir"]      = server_ds.s1["frames"]["webcam1"]

            if update_base_frame:
                update_base_frame = False
                server_ds.s1["frames"]["webcam1"] = base64.b64encode(cv2.imencode(".jpg", frame)[1]).decode()
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            server_ds.s1["hazmats_found"] = detection_ds.s2["found"]["hazmat"]
            server_ds.s1["qr_found"]      = detection_ds.s2["found"]["qr"]

            server_ds.s1["angle"] = detection_ds.s2["angle"]

            server_ds.s1["timebars"]["hazmat"] = time.time() - detection_ds.s2["last_update"]

            server_ds.s1["time"] = frame_read_times[base_key]

            if video_capture_zero:
                fpses = [
                    camera_sses[base_key].s["fps"],
                    detection_ds.s2["fps"],
                    camera_sses[base_key].s["fps"],
                    camera_sses[base_key].s["fps"],
                    fps_controller.fps(),
                    -1,
                    -1,
                ]
            else:
                fpses = [
                    camera_sses["webcam1"].s["fps"],
                    detection_ds.s2["fps"],
                    camera_sses["webcam2"].s["fps"],
                    camera_sses["ir"].s["fps"],
                    fps_controller.fps(),
                    -1,
                    -1,
                ]
            server_ds.s1["fpses"] = fpses

            server_ds.s1["ram"] = psutil.virtual_memory().percent
            server_ds.s1["cpu"] = psutil.cpu_percent()

            if gpu_log_file is not None:
                last_line = master.util.read_last_line(gpu_log_file)
                peices = last_line.split()
                for i, peice in enumerate(peices):
                    if peice == "GR3D_FREQ":
                        section = peices[i + 1]
                        server_ds.s1["gpu"] = float(section.split("%")[0])    
                        break        

            server_ds.put_s1(server_dq)
            # ---------------------------------------------------------------- #
    finally:
        if gpu_log_file is not None:
            gpu_log_file.close()