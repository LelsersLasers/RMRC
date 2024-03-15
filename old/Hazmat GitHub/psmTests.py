from __future__ import print_function
from levenshtein import *
import numpy as np 
import argparse
import mahotas
import pytesseract
import cv2

def removeSpecialCharacter(s):
    t = ""
    for i in s:
        if i >= 'A' and i <= 'Z':
            t += i
    return t

words = ["explosive", "blasting agent", "non flammable gas", "inhalation hazard", "infectious substance", "flammable liquid", 
"spontaneously combustible", "dangerous when wet", "oxidizer", "organic peroxide", "poison", "corrosive"]

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True,
    help = "Path to the image")
args = vars(ap.parse_args())

img = cv2.imread(args["image"])
cv2.imshow("image", img)


text = pytesseract.pytesseract.image_to_string(img, config="--psm 6")
text = removeSpecialCharacter(text)
print(text)
# if text == "":
#     text = pytesseract.pytesseract.image_to_string(img, config="--psm 6")
#     print("PSM 8 not applicable")
# if text != "":
#     text = removeSpecialCharacter(text)
#     # closest, distance = checkList(text, words)
#     print(text)
#     # print(closest)
#     # print(distance)

cv2.waitKey()