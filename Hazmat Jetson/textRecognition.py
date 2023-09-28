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
cv2.imshow("image", img)

text = pytesseract.pytesseract.image_to_string(img, config="--psm 6")
print(text)

cv2.waitKey(0)