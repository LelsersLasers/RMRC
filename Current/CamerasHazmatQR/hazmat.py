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
- rotate first
"""


def rotate(cropped):
    rows, cols = cropped.shape[:2]
    center = (cols / 2, rows / 2)

    # TODO: why these angels, and not 90, 180, 270 (every 45 degrees)?
    angles = [-45, 45, 135, -135]
    rotated = []
    for angel in angles:
        matrix = cv2.getRotationMatrix2D(center, angel, 1)
        rotated.append(cv2.warpAffine(cropped, matrix, (cols, rows)))

    rotated.append(cropped)

    return rotated


def processScreenshot(img, ratio_thresh, pool_size):
    # ------------------------------------------------------------------------ #
    h, w = img.shape[:2]

    mask = np.zeros_like(img)
    boxes = pytesseract.pytesseract.image_to_boxes(img)
    for box in boxes.splitlines():
        box = box.lower().strip().split()

        if len(box) != 6:
            continue

        text = box[0]
        if text == "~" or not text.isalpha():
            continue

        # TODO: is this right??
        x1 = int(box[1])
        y1 = int(box[2])
        x2 = int(box[3])
        y2 = int(box[4])

        w = x2 - x1
        h = y2 - y1
        
        x1 = int(x1 - w / 4)
        y1 = int(y1 - h / 4)
        x2 = int(x2 + w / 4)
        y2 = int(y2 + h / 4)

        cv2.rectangle(mask, (x1, y1), (x2, y2), (255, 255, 255), -1)

        print(box)

    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    imageList = []
    
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)

        cropped = img[y : y + h, x : x + w]
        rotated = rotate(cropped)

        for image in rotated:
            imageList.append((image, cnt))

    img_cnt = np.array([[0, 0], [0, img.shape[0]], [img.shape[1], img.shape[0]], [img.shape[1], 0]], dtype=np.int32)
    imageList.append((img, img_cnt))
    # ------------------------------------------------------------------------ #
    
    # ------------------------------------------------------------------------ #

    with Pool(pool_size) as pool:
        tesseract_results = pool.map(pytesseract.pytesseract.image_to_string, [image for image, _ in imageList])

        results = []
        for tesseract_result, (image, cnt) in zip(tesseract_results, imageList):
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

    return correct_tups, mask
    # ------------------------------------------------------------------------ #

