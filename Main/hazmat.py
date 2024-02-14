import cv2
import numpy as np
import scipy
import levenshtein
import util


class Rotated:
    def __init__(self, image, angle):
        self.image = image
        self.angle = angle

def rotate(image, angle):
    rotated_image = scipy.ndimage.rotate(image, angle)
    rotated = Rotated(rotated_image, angle)
    return rotated

def unrotate_cnt(cnt_rotated, rotated, img_shape):
    img_h, img_w = img_shape[:2]

    mask = np.zeros(rotated.image.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [cnt_rotated], -1, 255, -1)
    
    mask = scipy.ndimage.rotate(mask, -rotated.angle)
    h, w = mask.shape[:2]
    x = int((w - img_w) / 2)
    y = int((h - img_h) / 2)
    mask = mask[y:y+img_h, x:x+img_w]

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours[0]

def combine_nearby(detection_results):
    new_detection_results = []
    for i, detection_result_a in enumerate(detection_results):
        for j, detection_result_b in enumerate(detection_results):
            if i == j:
                continue

            if detection_result_a.overlaps(detection_result_b):
                new_detection_result = detection_result_a.combine(detection_result_b)
                new_detection_results.append(new_detection_result)
    
    for new_detection_result in new_detection_results:
        detection_results.append(new_detection_result)

def processScreenshot(img, angle, reader, levenshtein_thresh):
    # ------------------------------------------------------------------------ #
    rotated = rotate(img, angle)

    detection_results = []
    result = reader.readtext(rotated.image)
    for r in result:
        text = r[1]
        if len(text) < 1:
            continue

        confidence = r[2]        

        # ---------------------------------------------------------------- #
        cnt_rotated = np.array(r[0], dtype=np.int32)
        cnt = unrotate_cnt(cnt_rotated, rotated, img.shape)
        cnt = util.CNT(cnt, img.shape, True)
        # ---------------------------------------------------------------- #
    
        detection_result = util.DetectionResult(cnt, text, confidence)
        detection_results.append(detection_result)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    combine_nearby(detection_results) # support for 2 words per label
    combine_nearby(detection_results) # support for 3 words per label
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
        closest, distance = levenshtein.checkList(detection_result.text, words)
        ratio = distance / len(closest)
        if ratio <= levenshtein_thresh:
            levenshtein_result = util.LevenshteinResult(detection_result, closest, ratio)
            levenshtein_results.append(levenshtein_result)

    levenshtein_results = util.remove_dups(levenshtein_results, lambda x: x.detection_result.cnt)

    return levenshtein_results
    # ------------------------------------------------------------------------ #
