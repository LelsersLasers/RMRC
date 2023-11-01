import cv2
import numpy as np
import levenshtein

"""
TODO:
- multiline label detection
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
    for angel in range(0, 360, 45):
        matrix = cv2.getRotationMatrix2D(center, angel, 1)
        undo_matrix = cv2.getRotationMatrix2D(center, -angel, 1)
        rotated_image = cv2.warpAffine(img, matrix, img.shape[1::-1])

        rotated = Rotated(rotated_image, angel, matrix, undo_matrix)
        rotateds.append(rotated)

    return rotateds


def ocr_and_rotated(reader, rotated):
    return reader.readtext(rotated.image), rotated


def processScreenshot(img, reader, levenshtein_thresh, ocr_thresh):
    # ------------------------------------------------------------------------ #
    rotateds = rotate(img)

    result_tups = []
    for rotated in rotateds:
        result = reader.readtext(rotated.image)
        for r in result:
            confidence = r[2]
            if confidence < ocr_thresh:
                continue

            for i in range(4):
                r[0][i] = np.dot(r[0][i], rotated.undo_matrix)[:2]
            cnt = np.array(r[0], dtype=np.int32)
            text = r[1]
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
            tup = (closest, word, confidence, cnt)
            correct_tups.append(tup)

    return correct_tups
    # ------------------------------------------------------------------------ #
