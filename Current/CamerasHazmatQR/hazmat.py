import cv2
import numpy as np
import pytesseract
import mahotas
import util
import levenshtein


def rotate(cropped):
    rows, cols = cropped.shape[:2]
    center = (cols / 2, rows / 2)

    angles = [-45, 45, 135, -135]
    rotated = []
    for angel in angles:
        matrix = cv2.getRotationMatrix2D(center, angel, 1)
        rotated.append(cv2.warpAffine(cropped, matrix, (cols, rows)))

    rotated.append(cropped)

    return rotated


def processScreenshot(img, val, ratio_thresh):
    # ------------------------------------------------------------------------ #
    lowerThresh = np.array([0, 0, 0])  # lower thresh for black
    upperThresh = np.array([val, val, val])  # upper thresh for white

    # Create a binary mask based on the color threshold range
    mask = cv2.inRange(img, lowerThresh, upperThresh)

    # apply mask
    binary = np.zeros_like(img)
    binary[mask > 0] = 255

    img = cv2.bitwise_not(binary)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(img, (5, 5), 0)

    T = mahotas.thresholding.rc(blurred)
    thresh = img.copy()
    thresh[thresh > T] = 255
    thresh[thresh < 255] = 0
    thresh = cv2.bitwise_not(thresh)

    _, thresh = cv2.threshold(gray, 50, 255, 0)
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    contours, _ = cv2.findContours(thresh, 1, 2)

    imageList = []
            if w > 63:

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        ratio = float(w) / h
            if ratio >= 0.8 and ratio <= 1.2:

                cropped = img[y : y + h, x : x + w]
                # TODO: why bitwise_and?
                cropped = cv2.bitwise_and(cropped, img[y : y + h, x : x + w])
                rotated = rotate(cropped)

                for image in rotated:
                    imageList.append((image, cnt))
    # ------------------------------------------------------------------------ #
    
    # ------------------------------------------------------------------------ #
    tesseract_results = []
    for image, cnt in imageList:
        width, height, _ = image.shape

        # TODO: how do these calculations work???
        x1 = int(0)
        y1 = int(height / 2 - (height * 0.15))
        x2 = int(width)
        y2 = int(height / 2 + (height * 0.25))

        onlyText = image[y1:y2, x1:x2]
        text = pytesseract.pytesseract.image_to_string(onlyText, config="--psm 6")
        text = util.removeSpecialCharacter(text)
        if text != "":
            tesseract_results.append((text, cnt))
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
    for tesseract_result in tesseract_results:
        word, cnt = tesseract_result
        closest, distance = levenshtein.checkList(word, words)
        ratio = distance / len(closest)
        if ratio <= ratio_thresh:
            tup = (closest, word, cnt)
            correct_tups.append(tup)

    return correct_tups
    # ------------------------------------------------------------------------ #

