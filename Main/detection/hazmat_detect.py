import numpy as np
import detection.util


def hazmat_detect(img, angle, reader, levenshtein_thresh):
    # ------------------------------------------------------------------------ #
    rotated = detection.util.Rotated.from_image_and_angle(img, angle)

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
        cnt = detection.util.CNT(cnt, img.shape, True)
        # ---------------------------------------------------------------- #
    
        detection_result = detection.util.DetectionResult(cnt, text, confidence)
        detection_results.append(detection_result)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    detection.util.combine_nearby(detection_results) # support for 2 words per label
    detection.util.combine_nearby(detection_results) # support for 3 words per label
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
        closest, distance = detection.levenshtein.checkList(detection_result.text, words)
        ratio = distance / len(closest)
        if ratio <= levenshtein_thresh:
            levenshtein_result = detection.util.LevenshteinResult(detection_result, closest, ratio)
            levenshtein_results.append(levenshtein_result)

    levenshtein_results = detection.util.remove_dups(levenshtein_results, lambda x: x.detection_result.cnt)

    return levenshtein_results