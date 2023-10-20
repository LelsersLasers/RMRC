import cv2
import numpy as np
import pytesseract
import mahotas
import util
import levenshtein


def processScreenshot(img, val, ratio_thresh):
    # CHANGE THRESHOLD AS NEEDED
    lowerThresh = np.array([0, 0, 0])  # lower thresh for black
    upperThresh = np.array([val, val, val])  # upper thresh for white

    # Create a binary mask based on the color threshold range
    mask = cv2.inRange(img, lowerThresh, upperThresh)

    # apply mask
    binary = np.zeros_like(img)
    binary[mask > 0] = 255

    img = cv2.bitwise_not(binary)

    # if args["debug"]:
    #     cv2.imshow("Inverted Binary Image", img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    # cv2.imshow("image", img)

    T = mahotas.thresholding.rc(blurred)
    thresh = img.copy()
    thresh[thresh > T] = 255
    thresh[thresh < 255] = 0
    thresh = cv2.bitwise_not(thresh)

    _, thresh = cv2.threshold(gray, 50, 255, 0)
    contours, _ = cv2.findContours(thresh, 1, 2)
    # print("Number of contours detected:", len(contours))

    imageList = []
    for i, cnt in enumerate(contours):
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w) / h
            if w > 63:
                if ratio >= 0.8 and ratio <= 1.2:
                    # img = cv2.drawContours(img, [cnt], -1, (255, 0, 0), 3)
                    # cv2.putText(
                    #     img, "Square", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
                    # )

                    # if args["debug"]:
                    #     cv2.imshow("squares", img)

                    cropped = img[y : y + h, x : x + w]
                    cropped = cv2.bitwise_and(cropped, img[y : y + h, x : x + w])
                    rows, cols = cropped.shape[:2]
                    matrixCW45 = cv2.getRotationMatrix2D((cols / 2, rows / 2), -45, 1)
                    cw45 = cv2.warpAffine(cropped, matrixCW45, (cols, rows))

                    matrixCCW45 = cv2.getRotationMatrix2D((cols / 2, rows / 2), 45, 1)
                    ccw45 = cv2.warpAffine(cropped, matrixCCW45, (cols, rows))

                    matrixCCW135 = cv2.getRotationMatrix2D((cols / 2, rows / 2), 135, 1)
                    ccw135 = cv2.warpAffine(cropped, matrixCCW135, (cols, rows))

                    matrixCW135 = cv2.getRotationMatrix2D((cols / 2, rows / 2), -135, 1)
                    cw135 = cv2.warpAffine(cropped, matrixCW135, (cols, rows))

                    imageList.append((cw45, cnt))
                    imageList.append((ccw45, cnt))
                    imageList.append((ccw135, cnt))
                    imageList.append((cw135, cnt))
                    imageList.append((cropped, cnt))
        # else:


    tesseract_results = []
    for i, (image, cnt) in enumerate(imageList):
        width, height, _ = image.shape

        # TODO: how do these calculations work???
        x1 = int(0)
        y1 = int(height / 2 - (height * 0.15))
        x2 = int(width)
        y2 = int(height / 2 + (height * 0.25))

        onlyText = image[y1:y2, x1:x2]
        # if args["debug"]:
        #     cv2.rectangle(image, (x1, y1), (x2, y2), (225, 0, 0), 2)
        #     cv2.imshow(f"image {i}", onlyText)
        text = pytesseract.pytesseract.image_to_string(onlyText, config="--psm 6")
        text = util.removeSpecialCharacter(text)
        if text != "":
            tesseract_results.append((text, cnt))

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
            tup = (closest, cnt)
            correct_tups.append(tup)

    return correct_tups
