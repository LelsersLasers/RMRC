import numpy as np
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
    help="Path to the image")
args = vars(ap.parse_args())

img = cv2.imread(args["image"])

cv2.imshow("original", img)

threshold = 100  # Adjust this value to increase/decrease the black point


# Increase the black point by setting a threshold
adjusted_img = np.where(img < threshold, 0, img)

cv2.imshow("black point", adjusted_img)

exposure = 10  # Adjust this value to increase/decrease the exposure

# Increase the exposure of the black point-adjusted image
adjusted_img = cv2.add(adjusted_img, exposure)

cv2.imshow("Exposure Adjusted Image", adjusted_img)

cv2.waitKey(0)
cv2.destroyAllWindows()
