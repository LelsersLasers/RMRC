#!/usr/bin/python3
from __future__ import print_function
from levenshtein import *
from cleanUp import *
from webcam import *
import numpy as np
import argparse
import mahotas
import pytesseract
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-d", "--debug", required=False, help="show debug windows", action="store_true")
args = vars(ap.parse_args())


def processScreenshot(img, val):
    # CHANGE THRESHOLD AS NEEDED
    lowerThresh = np.array([0, 0, 0])  # lower thresh for black
    upperThresh = np.array([val, val, val])  # upper thresh for white

    # Create a binary mask based on the color threshold range
    mask = cv2.inRange(img, lowerThresh, upperThresh)

    # apply mask
    binary = np.zeros_like(img)
    binary[mask > 0] = 255

    img = cv2.bitwise_not(binary)

    if args["debug"]:
        cv2.imshow("Inverted Binary Image", img)

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
        x1, y1 = cnt[0][0]
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w) / h
            if w > 63:
                count = i
                if ratio >= 0.8 and ratio <= 1.2:
                    img = cv2.drawContours(img, [cnt], -1, (255, 0, 0), 3)
                    cv2.putText(
                        img, "Square", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
                    )

                    if args["debug"]:
                        cv2.imshow("squares", img)

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

                    imageList.append((cw45, count, cnt))
                    imageList.append((ccw45, count, cnt))
                    imageList.append((ccw135, count, cnt))
                    imageList.append((cw135, count, cnt))
                    imageList.append((cropped, count, cnt))

    myDict = {}
    for i, (image, count, cnt) in enumerate(imageList):
        width, height, _ = image.shape

        # TODO: how do these calculations work???
        x1 = int(0)
        y1 = int(height / 2 - (height * 0.15))
        x2 = int(width)
        y2 = int(height / 2 + (height * 0.25))

        onlyText = image[y1:y2, x1:x2]
        if args["debug"]:
            cv2.rectangle(image, (x1, y1), (x2, y2), (225, 0, 0), 2)
            cv2.imshow(f"image {i}", onlyText)
        text = pytesseract.pytesseract.image_to_string(onlyText, config="--psm 6")
        text = removeSpecialCharacter(text)
        if text != "":
            myDict.update({text: (cnt)})

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
    for key in myDict:
        word = key
        closest, distance = checkList(word, words)
        ratio = distance / len(closest)
        if ratio <= 0.55:
            cnt = myDict[key]
            tup = (closest, cnt)
            correct_tups.append(tup)

    correct_tups = remove_dups(correct_tups, lambda x: x[0])
    return correct_tups


def remove_dups(list, comp):
    new_list = []
    for item in list:
        if comp(item) not in [comp(x) for x in new_list]:
            new_list.append(item)
    return new_list


def findMax(list):
    return max(list, key=len)


def hazmat_main():
    img = screenshot()

    threshVals = [90, 100, 110, 120, 130, 140, 150, 160, 170]
    found_this_frame = []
    for threshVal in threshVals:
        received_tups = processScreenshot(img, threshVal)

        for r in received_tups:
            found_this_frame.append(r)

    found_this_frame = remove_dups(found_this_frame, lambda x: x[0])

    print(f"\nFound {len(found_this_frame)} hazmat labels:")

    fontScale = 0.5
    fontColor = (0, 0, 255)
    thickness = 1
    lineType = 2

    for found in found_this_frame:
        text, cnt = found

        img = cv2.drawContours(img, [cnt], -1, (255, 0, 0), 3)
        x, y, w, h = cv2.boundingRect(cnt)

        print(text, "\t")

        cv2.rectangle(img, (x, y), (x + w, y + h), (225, 0, 0), 4)

        corner = (x + 5, y + 15)

        cv2.putText(
            img,
            text,
            corner,
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale,
            fontColor,
            thickness,
            lineType,
        )

    print("")

    cv2.imshow("hazmat image", img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    hazmat_main()
