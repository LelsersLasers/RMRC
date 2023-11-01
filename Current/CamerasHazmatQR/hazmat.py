import cv2
import numpy as np
import scipy
import levenshtein
import util


class Rotated:
    def __init__(self, image, angle):
        self.image = image
        self.angle = angle

def rotate(img):
    rotateds = []
    # TODO: 90 vs 45 (vs 60)
    for angle in range(0, 360, 90):
        rotated_image = scipy.ndimage.rotate(img, angle)
        rotated = Rotated(rotated_image, angle)
        rotateds.append(rotated)

    return rotateds

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

def processScreenshot(img, reader, levenshtein_thresh, ocr_thresh):
    # ------------------------------------------------------------------------ #
    rotateds = rotate(img)

    result_tups = []
    for rotated in rotateds:
        result = reader.readtext(rotated.image)
        for r in result:
            confidence = r[2]
            if confidence < ocr_thresh:
                print(r) # debug
                continue

            text = r[1]
            if len(text) < 2:
                continue

            # ---------------------------------------------------------------- #
            cnt_rotated = np.array(r[0], dtype=np.int32)
            cnt = unrotate_cnt(cnt_rotated, rotated, img.shape)
            cnt = util.CNT(cnt, img.shape)
            # ---------------------------------------------------------------- #
        
            result_tups.append((text, cnt, confidence))    
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    new_result_tups = []
    for i, result_tup_a in enumerate(result_tups):
        for j, result_tup_b in enumerate(result_tups):
            if i == j:
                continue

            text_a, cnt_a, conf_a = result_tup_a
            text_b, cnt_b, conf_b = result_tup_b

            if cnt_a == cnt_b:
                text = f'{text_a} {text_b}'
                cnt = cnt_a.combine(cnt_b)
                confidence = (conf_a + conf_b) / 2

                new_result_tups.append((text, cnt, confidence))
    
    for result_tup in new_result_tups:
        result_tups.append(result_tup)
    # ------------------------------------------------------------------------ #
    
    # ------------------------------------------------------------------------ #
    words = [
        "explosive",
        "blasting agent",
        "non flammable gas",
        "inhalation hazard",
        "infectious substance",
        "flammable liquid",
        "spontaneously combustible",
        "dangerous when wet",
        "oxidizer",
        "organic peroxide",
        "poison",
        "corrosive",
        "flammable gas",
    ]

    correct_tups = []
    for word, cnt, confidence in result_tups:
        closest, distance = levenshtein.checkList(word, words)
        ratio = distance / len(closest)
        if ratio <= levenshtein_thresh:
            tup = (closest, word, confidence, ratio, cnt)
            correct_tups.append(tup)
        else: # debug
            print(word, confidence, ratio)

    return correct_tups
    # ------------------------------------------------------------------------ #
