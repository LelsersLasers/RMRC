import time

import cv2
import easyocr

import shared_util

import detection.consts
import detection.util
import detection.levenshtein

import detection.hazmat_detect
import detection.qr_detect
import detection.motion_detect


# ---------------------------------------------------------------------------- #
def process(detection_dq):
    detection_ds = shared_util.DoubleState(detection.consts.STATE_FROM_SERVER, detection.consts.STATE_FROM_SELF)

    last_clear = detection.consts.STATE_FROM_SERVER["clear"].copy()
    hazmat_angle_change = detection.consts.STATE_FROM_SERVER["hazmat_angle_change"]

    levenshtein_results = {}

    print("Creating easyocr reader...")
    reader = easyocr.Reader(["en"], gpu=True)
    print("easyocr reader created.")

    fps_controller = shared_util.FPSController()
    graceful_killer = shared_util.GracefulKiller()

    while not graceful_killer.kill_now:
        detection_ds.update_s1(detection_dq)

        input_frame = detection_ds.s1["frame"]
        output_frame = input_frame.copy() if input_frame is not None else None

        # -------------------------------------------------------------------- #
        for key, value in detection_ds.s1["clear"].items():
            if value > last_clear[key]:
                last_clear[key] = value
                detection_ds.s2["found"][key] = []
                print(f"Cleared all found {key} labels.")
        # -------------------------------------------------------------------- #

        # -------------------------------------------------------------------- #
        if input_frame is not None:
            # ---------------------------------------------------------------- #
            if not detection_ds.s1["run"]["hazmat"] or detection_ds.s2["angle"] == 0:
                hazmat_angle_change = detection_ds.s1["hazmat_angle_change"]

            if detection_ds.s1["run"]["hazmat"] or detection_ds.s2["angle"] != 0:
                levenshtein_thresh = detection_ds.s1["hazmat_levenshtein_thresh"]
                frame_results = detection.hazmat_detect.hazmat_detect(input_frame, detection_ds.s2["angle"], reader, levenshtein_thresh)
                levenshtein_results[detection_ds.s2["angle"]] = frame_results

                fontScale = 0.25
                fontColor = (0, 0, 255)
                thickness = 1
                lineType = 2

                for results in levenshtein_results.values():
                    for result in results:
                        detection_ds.s2["found"]["hazmat"].append(result.closest)

                        output_frame = cv2.drawContours(output_frame, [result.detection_result.cnt.cnt], -1, (255, 0, 0), 2)

                        x, y, w, h = cv2.boundingRect(result.detection_result.cnt.cnt)
                        cv2.rectangle(output_frame, (x, y), (x + w, y + h), (0, 225, 0), 3)

                        corner = (x + 5, y - 10)

                        cv2.putText(
                            output_frame,
                            result.string,
                            corner,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale,
                            fontColor,
                            thickness,
                            lineType,
                        )

                if len(levenshtein_results[detection_ds.s2["angle"]]) > 0:
                    detection_ds.s2["found"]["hazmat"] = list(set(detection_ds.s2["found"]["hazmat"]))
                    detection_ds.s2["found"]["hazmat"].sort()

                    print([result.string for result in levenshtein_results[detection_ds.s2["angle"]]])
                    print(detection_ds.s2["found"]["hazmat"])

                detection_ds.s2["angle"] += hazmat_angle_change
                detection_ds.s2["angle"] %= 360
            else:
                detection_ds.s2["angle"] = 0
                levenshtein_results = {}
            # ---------------------------------------------------------------- #
        
            # ---------------------------------------------------------------- #
            if detection_ds.s1["run"]["qr"]:
                start = time.time()

                qr_found_this_frame = detection.qr_detect.qr_detect_and_draw(input_frame)
                if len(qr_found_this_frame) > 0:
                    previous_qr_count = len(detection_ds.s2["found"]["qr"])
                    for qr in qr_found_this_frame:
                        detection_ds.s2["found"]["qr"].append(qr.strip())

                    detection_ds.s2["found"]["qr"] = list(set(detection_ds.s2["found"]["qr"]))
                    detection_ds.s2["found"]["qr"].sort()

                    if len(detection_ds.s2["found"]["qr"]) > previous_qr_count:
                        print(qr_found_this_frame)
                        print(detection_ds.s2["found"]["qr"])

                detection_ds.s2["time_bars"]["qr"] = time.time() - start
            else:
                detection_ds.s2["time_bars"]["qr"] = -1
            # ---------------------------------------------------------------- #
            
            # ---------------------------------------------------------------- #
            if detection_ds.s1["run"]["motion"]:
                start = time.time()

                motion_min_area = detection_ds.s1["motion_min_area"]
                motion_threshold = detection_ds.s1["motion_threshold"]
                average_frame = detection_ds.s1["average_frame"]

                if average_frame is not None:
                    detection.motion_detect.motion_detect_and_draw(
                        input_frame,
                        average_frame,
                        output_frame,
                        motion_min_area,
                        motion_threshold
                    )
                
                detection_ds.s2["time_bars"]["motion"] = time.time() - start
            else:
                detection_ds.s2["time_bars"]["motion"] = -1
            # ---------------------------------------------------------------- #
        # -------------------------------------------------------------------- #

        # -------------------------------------------------------------------- #
        fps_controller.update()
        detection_ds.s2["fps"] = fps_controller.fps()

        for key, value in detection_ds.s2["found"].items():
            detection_ds.s2["found"][key] = list(set(value))
            detection_ds.s2["found"][key].sort()

        detection_ds.s2["last_update"] = time.time()
        detection_ds.s2["frame"] = output_frame

        detection_ds.put_s2(detection_dq)
        # -------------------------------------------------------------------- #

        # -------------------------------------------------------------------- #
        if not any(detection_ds.s1["run"].values()):
            time.sleep(1 / detection.consts.DRY_FPS)
        # -------------------------------------------------------------------- #
# ---------------------------------------------------------------------------- #