from PIL import Image
import numpy as np
import argparse
import pytesseract
import cv2
from binary import *

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
                help="Path to the image")
args = vars(ap.parse_args())

img = cv2.imread(args["image"])

img = binary(img)

cv2.imshow("imported binary", img)

# Convert NumPy array to PIL image
pil_image = Image.fromarray(img)

pil_image.show()

text = pytesseract.pytesseract.image_to_string(pil_image, config="--psm 6")
print(text)

# Display the image using OpenCV's imshow
cv2.waitKey(0)
