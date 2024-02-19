import time

import cv2
import numpy as np
import easyocr

import shared_util

import hazmat.consts
import hazmat.util
import hazmat.levenshtein


# ---------------------------------------------------------------------------- #
def processScreenshot(img, angle, reader, levenshtein_thresh):
    # ------------------------------------------------------------------------ #
    rotated = hazmat.util.Rotated.from_image_and_angle(img, angle)

    detection_results = []
    result = reader.readtext(rotated.image)
    for r in result:
        text = r[1]
        if len(text) < 1:
            continue

        confidence = r[2]        

        # ---------------------------------------------------------------- #
        cnt_rotated = np.array(r[0], dtype=np.int32)
        cnt = rotated.unrotate_cnt(cnt_rotated, img.shape)
        cnt = hazmat.util.CNT(cnt, img.shape, True)
        # ---------------------------------------------------------------- #
    
        detection_result = hazmat.util.DetectionResult(cnt, text, confidence)
        detection_results.append(detection_result)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    hazmat.util.combine_nearby(detection_results) # support for 2 words per label
    hazmat.util.combine_nearby(detection_results) # support for 3 words per label
    # ------------------------------------------------------------------------ #
    
    # ------------------------------------------------------------------------ #
    words = [
        "explosive",
        "blasting agent",
        "non flammable gas",
        "inhalation hazard",
        "infectious substance",
        "flammable liquid",
        "flammable solid",
        "spontaneously combustible",
        "dangerous when wet",
        "oxidizer",
        "organic peroxide",
        "poison",
        "corrosive",
        "flammable gas",
    ]

    levenshtein_results = []
    for detection_result in detection_results:
        closest, distance = hazmat.levenshtein.checkList(detection_result.text, words)
        ratio = distance / len(closest)
        if ratio <= levenshtein_thresh:
            levenshtein_result = hazmat.util.LevenshteinResult(detection_result, closest, ratio)
            levenshtein_results.append(levenshtein_result)

    levenshtein_results = hazmat.util.remove_dups(levenshtein_results, lambda x: x.detection_result.cnt)

    return levenshtein_results
    # ------------------------------------------------------------------------ #
# ---------------------------------------------------------------------------- #

# ---------------------------------------------------------------------------- #
def thread(hazmat_dq):
    fps_controller = shared_util.FPSController()

    all_found = []
    frame = hazmat.consts.STATE_FROM_MASTER["frame"]

    levenshtein_results = {}

    hazmat_ds = shared_util.DoubleState(hazmat.consts.STATE_FROM_MASTER, hazmat.consts.STATE_FROM_SELF)
    last_clear = hazmat.consts.STATE_FROM_MASTER["clear"]
    hazmat_angle_change = hazmat.consts.STATE_FROM_MASTER["hazmat_angle_change"]

    print("Creating easyocr reader...")
    reader = easyocr.Reader(["en"], gpu=True)
    print("easyocr reader created.")

    try:
        while not hazmat_ds.s1["quit"]:
            hazmat_ds.update_s1(hazmat_dq)

            # ---------------------------------------------------------------- #
            if hazmat_ds.s1["clear"] > last_clear:
                last_clear = hazmat_ds.s1["clear"]
                all_found = []
                print("Cleared all found hazmat labels.")
            # ---------------------------------------------------------------- #

            if hazmat_ds.s1["frame"] is not None:
                frame = hazmat_ds.s1["frame"]

                if not hazmat_ds.s1["run_hazmat"] or hazmat_ds.s2["angle"] == 0:
                    hazmat_angle_change = hazmat_ds.s1["hazmat_angle_change"]

                if hazmat_ds.s1["run_hazmat"] or hazmat_ds.s2["angle"] != 0:

                    levenshtein_thresh = hazmat_ds.s1["hazmat_levenshtein_thresh"]
                    frame_results = processScreenshot(frame, hazmat_ds.s2["angle"], reader, levenshtein_thresh)
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

                    hazmat_ds.s2["angle"] += hazmat_angle_change
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
                time.sleep(1 / hazmat.consts.HAZMAT_DRY_FPS)
            # ---------------------------------------------------------------- #
    except KeyboardInterrupt: pass
# ---------------------------------------------------------------------------- #