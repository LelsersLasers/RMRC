from __future__ import print_function
import numpy as np 
import argparse
import mahotas
import pytesseract
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True,
    help = "Path to the image")
args = vars(ap.parse_args())

img = cv2.imread(args["image"])

lowerBlack = np.array([0, 0, 200], dtype = "uint8") 
upperBlack = np.array([90, 90, 90], dtype = "uint8")

mask = cv2.inRange(img, upperBlack, lowerBlack)
detected_output = cv2.bitwise_and(img, img, mask =  mask) 
# whiteMask = cv2.bitwise_and(detected_output)

cv2.imshow("black detection", detected_output) 

cv2.waitKey(0) 

