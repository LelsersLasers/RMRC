import numpy as np
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
                help="Path to the image")
args = vars(ap.parse_args())

# Read the image
img = cv2.imread(args["image"])

cv2.imshow("original", img)

# Define the exposure adjustment value
exposure = 20  # Adjust this value to increase/decrease the exposure

# Increase the exposure of the image
adjusted_img = cv2.add(img, exposure)

# Define the contrast adjustment value
contrast = 1.5  # Adjust this value to increase/decrease the contrast

# Adjust the contrast of the image
adjusted_img = cv2.multiply(adjusted_img, contrast)

# Apply Gaussian blur to the adjusted image
blurred = cv2.GaussianBlur(adjusted_img, (0, 0), 3)

# Calculate the sharpened image using unsharp masking
unsharp_image = cv2.addWeighted(adjusted_img, 1.5, blurred, -0.5, 0)

# Display the final adjusted image with exposure, contrast, and sharpness
cv2.imshow("Final Adjusted Image", unsharp_image)

# Wait for a key press and then close all windows
cv2.waitKey(0)
cv2.destroyAllWindows()
