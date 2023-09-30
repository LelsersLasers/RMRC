from __future__ import print_function
import numpy as np 
import argparse 
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True,
    help = "Path to the image")
args = vars(ap.parse_args())

image = cv2.imread(args["image"])
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11, 11), 0)
cv2.imshow("Image", image)

edged = cv2.Canny(blurred, 30, 150)
cv2.imshow("Edges", edged)
# find contours of coins
# second argument is type of contours we want (external is only the outline)
# third arugmnet is way of approximating contour
# in this case, we're compressing the horizontal, vertical, and 
# diagonal segments into their endpoints only
(cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)

print("I count {} coins in this image".format(len(cnts)))

coins = image.copy()
# -1 says you want to draw ALL the contours
cv2.drawContours(coins, cnts, -1, (200, 255, 0), 2)
cv2.imshow("Coins", coins)

for (i, c) in enumerate(cnts):
    (x, y, w, h) = cv2.boundingRect(c)

    print("Coin#{}".format(i+1))
    coin = image[y:y + h, x:x + w]
    cv2.imshow("Coin", coin)

    mask = np.zeros(image.shape[:2], dtype = "uint8")
    ((centerX, centerY), radius) = cv2.minEnclosingCircle(c)
    cv2.circle(mask, (int(centerX), int(centerY)), int(radius), 255, -1)
    mask = mask[y:y +h, x:x + w]
    cv2.imshow("Masked Coin{}".format(i+1), cv2.bitwise_and(coin, coin, mask = mask))

cv2.waitKey(0)