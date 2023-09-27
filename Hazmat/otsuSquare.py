from __future__ import print_function
import numpy as np 
import argparse
import mahotas
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True,
    help = "Path to the image")
args = vars(ap.parse_args())

img = cv2.imread(args["image"])
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(img, (5, 5), 0)
cv2.imshow("image", img)

T = mahotas.thresholding.otsu(blurred)
print("tsu's threshold: {}.format(T)")
thresh = img.copy()
thresh[thresh > T] = 255
thresh[thresh < 255] = 0
img = cv2.bitwise_not(thresh)
cv2.imshow("test", img)

ret,thresh = cv2.threshold(img,50,255,0)
contours,hierarchy = cv2.findContours(thresh, 1, 2)
print("Number of contours detected:", len(contours))

for cnt in contours:
   x1,y1 = cnt[0][0]
   approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
   if len(approx) == 4:
      x, y, w, h = cv2.boundingRect(cnt)
      ratio = float(w)/h
      if w > 3:
        if ratio >= 0.8 and ratio <= 1.2:
            print("Length of w:",w)
            print("Ratio:", ratio)
            img = cv2.drawContours(thresh, [cnt], -1, (0,255,255), 3)
            cv2.putText(img, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
cv2.imshow("Shapes", img)

cv2.waitKey(0)