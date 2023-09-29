cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"


import cv2
import time
from levenshtein import *
from cleanUp import *
import numpy as np 
import argparse
import mahotas
import pytesseract

ap = argparse.ArgumentParser()
ap.add_argument("-d", "--debug", required=False, help="show debug windows", action="store_true")
# ap.add_argument(
#     "-n",
#     "--number",
#     required=False,
#     help="number of hazmat labels you see",
#     default=None,
# )
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

                    if args["debug"]:
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
        if args["debug"]:
            cv2.imshow(f"image {i}", onlyText)
        text = pytesseract.pytesseract.image_to_string(onlyText, config="--psm 6")
        text = removeSpecialCharacter(text)
        # if text == "":
        #     text = pytesseract.pytesseract.image_to_string(onlyText, config="--psm 6")
        if text != "":
            # print(f"Image to string for image {i}: {text}")
            myDict.update({text:(x1, y1, x2, y2)})

    # print(myDict)
    words = ["explosive", "blasting agent", "non flammable gas", "inhalation hazard", "infectious substance", "flammable liquid", 
    "spontaneously combustible", "dangerous when wet", "oxidizer", "organic peroxide", "poison", "corrosive", "flammable gas"]
    # correct = []
    correct_tups = []
    for key in myDict:  
        word = key
        closest, distance = checkList(word, words)
        ratio = distance/len(closest)
        if ratio <= 0.55:
            x1, y1, x2, y2  = myDict[key]
            
            tup = (closest, x1, y1, x2, y2)

            # correct.append(closest)
            correct_tups.append(tup)


            # print(f"for image #{myDict[key]}")
            # print(f"the starting word is {key}", end="")
            # print()
            # print(f"the closest value is {closest}")
            # # print(f"the distance is {distance}")
            # # print(f"ratio: {ratio}")
            # print()
    # correct = list(set(correct))
    # return correct
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


def main():
    print("Press 'q' to close.")

    cap = cv2.VideoCapture(cap_args, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        raise RuntimeError("Can't open camera. Are the cap_args set right? Is the camera plugged in?")

    time.sleep(1)
     
    all_found = []

    t0 = time.time()
    t1 = time.time()
    delta = 1 / 30

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Exiting ...")
            break
          
        threshVals = [90, 100, 110, 120, 130, 140, 150, 160, 170] 
        found_this_frame = []
        for threshVal in threshVals:
            received_tups = processScreenshot(frame, threshVal)
            # print(f"for threshVal of {threshVal}: {received}")

            for r in received_tups:
                found_this_frame.append(r)
                all_found.append(r[0])

        found_this_frame = remove_dups(found_this_frame, lambda x: x[0])
        all_found = list(set(all_found))


        fontScale = 0.5
        fontColor = (0, 0, 255)
        thickness = 1
        lineType = 2

        for found in found_this_frame:
            text, x1, y1, x2, y2 = found
            cv2.rectangle(frame, (x1, y1), (x2, y2), (225,0,0), 2)

            corner = (x1, y1 - 5)

            cv2.putText(
                frame,
                text,
                corner,
                cv2.FONT_HERSHEY_SIMPLEX,
                fontScale,
                fontColor,
                thickness,
                lineType,
            )


        print("\n")
        print([x[0] for x in found_this_frame])
        print(all_found)
        print("FPS: %.2f" % (1 / delta))


        t1 = time.time()
        delta = t1 - t0
        t0 = t1

        cv2.imshow("Camera feed + auto hazmat", frame)


        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break



    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

