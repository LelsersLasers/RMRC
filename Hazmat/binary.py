from __future__ import print_function
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
def binary(img):
    lowerThresh = np.array([0, 0, 0])  # lower thresh for black
    upperThresh = np.array([150, 150, 150])  # upper thresh for white

    # Create a binary mask based on the color threshold range
    mask = cv2.inRange(img, lowerThresh, upperThresh)

    # apply mask
    binary = np.zeros_like(img)
    binary[mask > 0] = 255

    inverted_binary = cv2.bitwise_not(binary)

    cv2.imshow('binary', inverted_binary)
    return inverted_binary
# cv2.waitKey(0)