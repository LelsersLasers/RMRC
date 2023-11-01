import cv2
import numpy as np
import levenshtein

"""
TODO:
- multiline label detection
- unrotate correctly
- 90 vs 45
    - likely bigger angle -> lower ocr_thresh
        - lower ocr_thresh isn't terrible on performance
    - smaller angle is harder on performance
"""


class Rotated:
    def __init__(self, image, angle, matrix, undo_matrix):
        self.image = image
        self.angle = angle
        self.matrix = matrix
        self.undo_matrix = undo_matrix

def rotate(img):
    center = tuple(np.array(img.shape[1::-1]) / 2)

    rotateds = []
    # TODO: 90 vs 45
    for angel in range(0, 360, 90):
        matrix = cv2.getRotationMatrix2D(center, angel, 1)
        undo_matrix = cv2.getRotationMatrix2D(center, -angel, 1)
        rotated_image = cv2.warpAffine(img, matrix, img.shape[1::-1])

        rotated = Rotated(rotated_image, angel, matrix, undo_matrix)
        rotateds.append(rotated)

    return rotateds


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
            if len(text) < 3:
                continue

            # TODO: unrotate correctly!
            # for i in range(4):
            #     r[0][i] = np.dot(r[0][i], rotated.undo_matrix)[:2]
            # cnt = np.array(r[0], dtype=np.int32)

            cnt = np.array(r[0], dtype=np.int32)

            mask = np.zeros(rotated.image.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask, [cnt], -1, 255, -1)
            cv2.warpAffine(mask, rotated.undo_matrix, mask.shape[1::-1])
            # mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnt = contours[0]
        
            result_tups.append((text, cnt, confidence))    
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
