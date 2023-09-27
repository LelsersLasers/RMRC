from __future__ import print_function
import numpy as np
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
args = vars(ap.parse_args())

rawImage = cv2.imread(args["image"])

bilateral_filtered_image = cv2.bilateralFilter(rawImage, 5, 175, 175)
cv2.imshow('Bilateral', bilateral_filtered_image)
edge_detected_image = cv2.Canny(bilateral_filtered_image, 75, 200)
cv2.imshow('Edge', edge_detected_image)
_, contours = cv2.findContours(edge_detected_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contour_list = []
for contour in contours:
    approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
    area = cv2.contourArea(contour)
    if ((len(approx) > 8) & (area > 30) ):
        contour_list.append(contour)
cv2.drawContours(rawImage, contour_list,  -1, (255,0,0), 2)
cv2.imshow('Objects Detected',rawImage)


cv2.waitKey(0)