import cv2
import numpy as np
import pytesseract
import mahotas
import util
import levenshtein
import time
from multiprocessing import Pool

"""
TODO:
- Not create pool twice
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

def mask_on_rotated(rotated):
    mask = np.zeros_like(rotated.image)
    boxes = pytesseract.pytesseract.image_to_boxes(rotated.image)

    print("\n")

    max_h = rotated.image.shape[0]

    for box in boxes.splitlines():
        box = box.lower().strip().split()

        if len(box) != 6:
            continue

        text = box[0]
        if text == "~" or not text.isalpha():
            continue

        x1 = int(box[1])
        y1 = max_h - int(box[2])
        x2 = int(box[3])
        y2 = max_h - int(box[4])

        w = x2 - x1
        h = y2 - y1
        
        x1 = int(x1 - w / 2)
        y1 = int(y1 - h / 2)
        x2 = int(x2 + w / 2)
        y2 = int(y2 + h / 2)

        print(box)

        cv2.rectangle(mask, (x1, y1), (x2, y2), (255, 255, 255), -1)

    mask = cv2.warpAffine(mask, rotated.undo_matrix, (rotated.image.shape[1], rotated.image.shape[0]))
    return mask


def processScreenshot(img, ratio_thresh, pool_size):
    # ------------------------------------------------------------------------ #
    with Pool(pool_size) as pool:
        rotateds = rotate(img)
        masks = pool.map(mask_on_rotated, rotateds)

        overall_mask = np.zeros_like(img)
        for mask in masks:
            overall_mask = cv2.bitwise_or(overall_mask, mask)


        overall_mask = cv2.cvtColor(overall_mask, cv2.COLOR_BGR2GRAY)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    contours, _ = cv2.findContours(overall_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    imageList = []
    
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)

        cropped = img[y : y + h, x : x + w]
        rotateds = rotate(cropped)

        for rotated in rotateds:
            imageList.append((rotated.image, cnt))
    # ------------------------------------------------------------------------ #
    
    # ------------------------------------------------------------------------ #
    with Pool(pool_size) as pool:
        tesseract_results = pool.map(pytesseract.pytesseract.image_to_string, [image for image, _ in imageList])

        results = []
        for tesseract_result, (_, cnt) in zip(tesseract_results, imageList):
            text = util.removeSpecialCharacter(tesseract_result)
            if text != "":
                results.append((text, cnt))
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
    for word, cnt in results:
        closest, distance = levenshtein.checkList(word, words)
        ratio = distance / len(closest)
        if ratio <= ratio_thresh:
            tup = (closest, word, cnt)
            correct_tups.append(tup)

    return correct_tups, overall_mask
    # ------------------------------------------------------------------------ #

