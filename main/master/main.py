import time
import base64

import cv2
import numpy as np
import psutil

import shared_util

import master.consts
import master.util

import detection.consts
import server.primary_server.consts
import camera.consts



def to_bs64(frame):
    return base64.b64encode(cv2.imencode(".jpg", frame)[1]).decode()


def process(detection_dq, primary_server_dq, camera_sqs, video_capture_zero):
    print("\nControls:")
    print("Hold 'wasd' or arrow keys to move the robot.")
    print("Hold 'z' to set all motor speeds to 0.")
    print("Press 't'/'T' to increase/decrease power by 20%.")
    
    print("Press 'h' to toggle running hazmat detection.")
    print("Press 'r' to toggle running qr detection.")
    print("Press 'm' to toggle running motion detection.")
    
    print("Press 'c' to clear all found hazmat labels.")
    print("Press 'x' to clear all found QR codes.")
    
    print("Press 'l' to invert controls.")
    print("Press 'p' to toggle showing detections output and IR.")
    print("Press 'o' to toggle arm active.")
    
    print("Press 1-4 to switched focused feed.")
    print("Press 1-4 again to super zoom on focused feed.")
    print("Press 0 to show camera grid.")
    print("Press 5 to toggle sidebar.\n")

    print("\npsutil info:")
    master.util.pretty_print_dict(psutil.cpu_count())
    master.util.pretty_print_dict(psutil.cpu_freq())
    master.util.pretty_print_dict(psutil.virtual_memory())
    master.util.pretty_print_dict(psutil.swap_memory())
    master.util.pretty_print_dict(psutil.net_if_stats())
    master.util.pretty_print_dict(psutil.sensors_temperatures())
    master.util.pretty_print_dict(psutil.sensors_fans())
    master.util.pretty_print_dict(psutil.sensors_battery())
    print("\n")


    average_frame = None
    last_frame = None

    detection_ds = shared_util.DoubleState(detection.consts.STATE_FROM_SERVER, detection.consts.STATE_FROM_SELF)
    primary_server_ds = shared_util.DoubleState(server.primary_server.consts.STATE_FROM_MASTER, server.primary_server.consts.STATE_FROM_SELF)

    gpu_log_file = None if video_capture_zero else open(master.consts.GPU_LOG_FILENAME, 'rb')

    camera_sses = {}
    frame_read_times = {}
    for key in camera_sqs.keys():
        camera_ss = shared_util.SingleState(camera.consts.STATE_FROM_SELF)
        camera_sses[key] = camera_ss
        frame_read_times[key] = 0

    last_detection_time = 0

    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()

    times = []

    try:
        while not graceful_killer.kill_now:
            start = time.time()
            fps_controller.update()

            # ---------------------------------------------------------------- #
            primary_server_ds.update_s2(primary_server_dq)

            base_key = None if video_capture_zero else ("webcam1" if not primary_server_ds.s2["invert"] else "webcam2")
            alt_key  = None if video_capture_zero else ("webcam2" if not primary_server_ds.s2["invert"] else "webcam1")
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            frames = {}
            for key, camera_sq in camera_sqs.items():
                camera_ss = camera_sses[key]
                camera_ss.update_s(camera_sq)
                frames[key] = camera_ss.s["frame"]

                if frames[key] is None:
                    frames[key]  = np.zeros((camera.consts.CAMERA_SIZE[1], camera.consts.CAMERA_SIZE[0], 3), dtype=np.uint8)
                    frames[key] += camera.consts.CAMERA_NONE_GREY

                if camera_ss.s["time"] > frame_read_times[key]:
                    frame_read_times[key] = camera_ss.s["time"]

                    if key == base_key:
                        primary_server_ds.s1["frames"]["webcam1"] = to_bs64(frames[key])

                        if average_frame is None or last_frame is None:
                            average_frame = frames[key].copy().astype("float")
                            last_frame = frames[key]
                        else:
                            motion_new_frame_weight = primary_server_ds.s2["motion_new_frame_weight"]
                            cv2.accumulateWeighted(last_frame.astype("float"), average_frame, motion_new_frame_weight)
                        last_frame = frames[key]
                    elif key == alt_key:
                        primary_server_ds.s1["frames"]["webcam2"] = to_bs64(frames[key])
                    elif key == "arm":
                        primary_server_ds.s1["frames"][key] = to_bs64(frames[key])
                    elif key == "ir" and not primary_server_ds.s2["show_detections"]:
                        primary_server_ds.s1["frames"]["detection_ir"] = to_bs64(frames[key])

            frame = frames[base_key]            
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            detection_ds.update_s2(detection_dq)

            if primary_server_ds.s2["show_detections"] and detection_ds.s2["last_update"] > last_detection_time:
                last_detection_time = detection_ds.s2["last_update"]

                if detection_ds.s2["frame"] is not None:
                    detection_frame = detection_ds.s2["frame"]
                else:
                    detection_frame = np.zeros_like(frame)

                primary_server_ds.s1["frames"]["detection_ir"] = to_bs64(detection_frame)
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            detection_ds.s1["run"] = primary_server_ds.s2["run"]
            detection_ds.s1["clear"] = primary_server_ds.s2["clear"]

            detection_ds.s1["hazmat_levenshtein_thresh"] = primary_server_ds.s2["hazmat_levenshtein_thresh"]
            detection_ds.s1["hazmat_angle_change"] = primary_server_ds.s2["hazmat_angle_change"]

            detection_ds.s1["motion_min_area"] = primary_server_ds.s2["motion_min_area"]
            detection_ds.s1["motion_threshold"] = primary_server_ds.s2["motion_threshold"]
            detection_ds.s1["motion_new_frame_weight"] = primary_server_ds.s2["motion_new_frame_weight"]

            detection_ds.s1["frame"] = frame
            detection_ds.s1["average_frame"] = average_frame.copy()
            detection_ds.put_s1(detection_dq)
            # ---------------------------------------------------------------- #

            # -----------------------------------------------------------------#
            if video_capture_zero:
                primary_server_ds.s1["frames"]["webcam2"] = primary_server_ds.s1["frames"]["webcam1"]
                primary_server_ds.s1["frames"]["arm"]     = primary_server_ds.s1["frames"]["webcam1"]
                if not primary_server_ds.s2["show_detections"]:
                    primary_server_ds.s1["frames"]["detection_ir"] = primary_server_ds.s1["frames"]["webcam1"]
            # ---------------------------------------------------------------- #

            # ---------------------------------------------------------------- #
            primary_server_ds.s1["hazmats_found"] = detection_ds.s2["found"]["hazmat"]
            primary_server_ds.s1["qr_found"]      = detection_ds.s2["found"]["qr"]

            primary_server_ds.s1["angle"] = detection_ds.s2["angle"]

            primary_server_ds.s1["time_bars"]["hazmat"] = time.time() - detection_ds.s2["last_update"]
            primary_server_ds.s1["time_bars"]["qr"] = detection_ds.s2["time_bars"]["qr"]
            primary_server_ds.s1["time_bars"]["motion"] = detection_ds.s2["time_bars"]["motion"]
            

            primary_server_ds.s1["time"] = frame_read_times[base_key]

            # "webcam1", "webcam2", "arm", "detection", "ir", "master", "motor", "armreader", "backend"
            if video_capture_zero:
                fpses = [
                    camera_sses[base_key].s["fps"],
                    camera_sses[base_key].s["fps"],
                    camera_sses[base_key].s["fps"],
                    detection_ds.s2["fps"],
                    camera_sses[base_key].s["fps"],
                    fps_controller.fps(),
                    -1,
                    -1,
                    -1,
                ]
            else:
                fpses = [
                    camera_sses["webcam1"].s["fps"],
                    camera_sses["webcam2"].s["fps"],
                    camera_sses["arm"].s["fps"],
                    detection_ds.s2["fps"],
                    camera_sses["ir"].s["fps"],
                    fps_controller.fps(),
                    -1,
                    -1,
                    -1,
                ]
            primary_server_ds.s1["fpses"] = fpses

            primary_server_ds.s1["stats"]["ram"] = psutil.virtual_memory().percent
            primary_server_ds.s1["stats"]["swap"] = psutil.swap_memory().percent
            primary_server_ds.s1["stats"]["cpu"] = psutil.cpu_percent()

            if gpu_log_file is not None:
                pieces_needed = 3
                last_line = master.util.read_last_line(gpu_log_file)
                pieces = last_line.split()
                for i, piece in enumerate(pieces):
                    if piece == "GR3D_FREQ":
                        section = pieces[i + 1]
                        primary_server_ds.s1["stats"]["gpu"] = float(section.split("%")[0])
                        pieces_needed -= 1
                    elif piece.startswith("CPU@"):
                        primary_server_ds.s1["temps"]["cpu"] = float(piece.split("@")[1].split("C")[0])
                        pieces_needed -= 1
                    elif piece.startswith("GPU@"):
                        primary_server_ds.s1["temps"]["gpu"] = float(piece.split("@")[1].split("C")[0])
                        pieces_needed -= 1
                    if pieces_needed == 0: break


            primary_server_ds.put_s1(primary_server_dq)

            pass_time = time.time() - start
            times.append(pass_time)
            times = times[-master.consts.TIMES_TO_KEEP:]
            avg_time = sum(times) / len(times)


            target_time = 1 / primary_server_ds.s2["master_fps"]
            sleep_target = target_time - avg_time
            if sleep_target > 0:
                time.sleep(sleep_target)
            # ---------------------------------------------------------------- #
    finally:
        if gpu_log_file is not None:
            gpu_log_file.close()