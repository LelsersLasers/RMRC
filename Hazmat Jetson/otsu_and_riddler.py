from __future__ import print_function
import numpy as np 
import argparse
import mahotas
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True,
    help = "Path to the image")
args = vars(ap.parse_args())

image = cv2.imread(args["image"])
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(image, (5, 5), 0)
cv2.imshow("Image", image)

T = mahotas.thresholding.otsu(blurred)
print("tsu's threshold: {}.format(T)")
thresh = image.copy()
thresh[thresh > T] = 255
thresh[thresh < 255] = 0
thresh = cv2.bitwise_not(thresh)

T = mahotas.thresholding.rc(blurred)
print("Riddler-Calvard: {}".format(T))
thresh = image.copy()
thresh[thresh > T] = 255
thresh[thresh < 255] = 0
img = cv2.bitwise_not(thresh)
cv2.imshow("Riddler-Calvard", img)

ret,thresh = cv2.threshold(img,50,255,0)
contours,hierarchy = cv2.findContours(thresh, 1, 2)
print("Number of contours detected:", len(contours))
cv2.waitKey(0)