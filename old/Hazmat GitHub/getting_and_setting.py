from __future__ import print_function
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True,
    help = "Path to the image")
args = vars(ap.parse_args())

image = cv2.imread(args["image"])
cv2.imshow("Original", image)

#why does OpenCV store RGB channels in reverse order?

#get pixel located at top left corner (repped as a tuple)
(b, g, r) = image[0,0]
print("Pixel at (0,0) - Red: {}, Green {}, Blue: {}".format(r,g,b))
#manipulate top left pixel to red color 
image[0,0] = (0, 0, 255)
(b,g,r) = image[0,0]
print("Pixel at (0,0) - Red: {}, Green: {}, Blue: {}".format(r, g, b))

#take an 100 by 100 pixel region (start x - end x, start y - end y)
corner = image[0:100, 0:100]
cv2.imshow("Corner", corner)

image[0:100, 0:100] = (0,255,0)

cv2.imshow("Updated", image)
cv2.waitKey(0)