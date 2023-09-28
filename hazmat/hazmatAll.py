#!/usr/bin/python3
from __future__ import print_function
from levenshtein import *
from cleanUp import *
#from screenshot import *
from opencv_webcam1 import *
import numpy as np 
import argparse
import mahotas
import pytesseract
import cv2

# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", required = True,
#     help = "Path to the image")
# args = vars(ap.parse_args())

# img = cv2.imread(args["image"])
def processScreenshot(img, val):

    cv2.imwrite("picamera_img.jpg", img)

    # CHANGE THRESHOLD AS NEEDED
    lowerThresh = np.array([0, 0, 0])  # lower thresh for black
    upperThresh = np.array([val, val, val])  # upper thresh for white

    # Create a binary mask based on the color threshold range
    mask = cv2.inRange(img, lowerThresh, upperThresh)

    # apply mask
    binary = np.zeros_like(img)
    binary[mask > 0] = 255

    img = cv2.bitwise_not(binary)

    cv2.imshow('Inverted Binary Image', img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    # cv2.imshow("image", img)

    T = mahotas.thresholding.rc(blurred)
    thresh = img.copy()
    thresh[thresh > T] = 255
    thresh[thresh < 255] = 0
    thresh = cv2.bitwise_not(thresh)

    ret,thresh = cv2.threshold(gray,50,255,0)
    contours,hierarchy = cv2.findContours(thresh, 1, 2)
    # print("Number of contours detected:", len(contours))

    imageList = []
    for i, cnt in enumerate(contours):
        x1,y1 = cnt[0][0]
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w)/h
            if w > 63:
                count = i
                if ratio >= 0.8 and ratio <= 1.2:
                    # print("Length of w:",w)
                    # print("Ratio:", ratio)
                    img = cv2.drawContours(img, [cnt], -1, (255,0,0), 3)
                    cv2.putText(img, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                    cv2.imshow("squares", img)
                    cropped = img[y:y+h, x:x+w]
                    cropped = cv2.bitwise_and(cropped, img[y:y+h, x:x+w])
                    rows, cols = cropped.shape[:2]
                    matrixCW45 = cv2.getRotationMatrix2D((cols/2, rows/2), -45, 1)
                    cw45 = cv2.warpAffine(cropped, matrixCW45, (cols, rows))
                    # cv2.imshow(f"45 cw {count}", cw45)

                    #only need for testing purposes
                    # matrix90 = cv2.getRotationMatrix2D((cols/2, rows/2), -90, 1)
                    # cw90 = cv2.warpAffine(cropped, matrix90, (cols, rows))
                    # cv2.imshow(f"90 cw {count}", cw90)

                    matrixCCW45 = cv2.getRotationMatrix2D((cols/2, rows/2), 45, 1)
                    ccw45 = cv2.warpAffine(cropped, matrixCCW45, (cols, rows))
                        # cv2.imshow(f"45 ccw {count}", ccw45)

                        #only need for testing purposes
                        # matrix270 = cv2.getRotationMatrix2D((cols/2, rows/2), 180, 1)
                        # cw180 = cv2.warpAffine(cropped, matrix270, (cols, rows))
                        # cv2.imshow(f"180 rotation {count}", cw180)

                    matrixCCW135 = cv2.getRotationMatrix2D((cols/2, rows/2), 135, 1)
                    ccw135 = cv2.warpAffine(cropped, matrixCCW135, (cols, rows))
                        # cv2.imshow(f"135 ccw {count}", ccw135)

                    matrixCW135 = cv2.getRotationMatrix2D((cols/2, rows/2), -135, 1)
                    cw135 = cv2.warpAffine(cropped, matrixCW135, (cols, rows))
                        # cv2.imshow(f"135 cw {count}", cw135)
                    imageList.append(cw45)
                    imageList.append(ccw45)
                    imageList.append(ccw135)
                    imageList.append(cw135)  
                    imageList.append(cropped)
            
    myDict = {}
    for i, image in enumerate(imageList):
        width, height, _ = image.shape
        x1 = int(0)
        y1 = int(height/2 - (height*0.15))
        x2 = int(width)
        y2 = int(height/2 + (height*0.25))
        cv2.rectangle(image, (x1, y1), (x2, y2), (225,0,0), 2)
        onlyText = image[y1:y2, x1:x2]
        cv2.imshow(f"image {i}", onlyText)
        text = pytesseract.pytesseract.image_to_string(onlyText, config="--psm 6")
        text = removeSpecialCharacter(text)
        # if text == "":
        #     text = pytesseract.pytesseract.image_to_string(onlyText, config="--psm 6")
        if text != "":
            # print(f"Image to string for image {i}: {text}")
            myDict.update({text:i})

    # print(myDict)
    words = ["explosive", "blasting agent", "non flammable gas", "inhalation hazard", "infectious substance", "flammable liquid", 
    "spontaneously combustible", "dangerous when wet", "oxidizer", "organic peroxide", "poison", "corrosive", "flammable gas"]
    correct = []
    for key in myDict:  
        word = key
        closest, distance = checkList(word, words)
        ratio = distance/len(closest)
        if ratio <= 0.55:
            correct.append(closest)
            # print(f"for image #{myDict[key]}")
            # print(f"the starting word is {key}", end="")
            # print()
            # print(f"the closest value is {closest}")
            # # print(f"the distance is {distance}")
            # # print(f"ratio: {ratio}")
            # print()
    correct = list(set(correct))
    return correct

def findMax(list):
    return max(list, key=len)

def main():
    # Capture the screenshot
    expected = int(input("Enter number of hazmat labels you see: "))
    img = screenshot()
    cv2.imshow("hazmat image", img)
    cv2.waitKey(3000)

    received = []
    threshVals = [90, 100, 110, 120, 130, 140, 150, 160, 170] 
    count = 0
    breakOrNot = False
    allReceived = []
    for i in threshVals:
        received = processScreenshot(img, threshVals[count])
        print(f"for threshVal of {threshVals[count]}: {received}")
        count += 1
        allReceived.append(received)
    #find trial with the most hazmat labels recorded
    longest = findMax(allReceived)
    print(f"the maximum amount of labels detected is {len(longest)}")
    for i in longest:
        print(i)

    

if __name__ == "__main__":
    main()
    cv2.waitKey(0)
