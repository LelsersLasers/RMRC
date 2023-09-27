import numpy as np 
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True,
    help = "Path to the image")
args=vars(ap.parse_args())

image = cv2.imread(args["image"])
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#help review noisy, unrealted edges
image = cv2.GaussianBlur(image, (5,5), 0)
cv2.imshow("Blurred", image)

#any gradient value larger than threshold2 (argument #2)
#is considered to be an edge
canny = cv2.Canny(image, 30, 150)
cv2.imshow("Canny", canny)
cv2.waitKey(0)