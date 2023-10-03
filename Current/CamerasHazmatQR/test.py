import cv2
import numpy as np

frame = np.zeros((512,512,3), np.uint8)

# text = "https://www.geeksforgeeks.org/python-opencv-cv2-puttext-method/"
# text = "https://en.wikipedia.org/wiki/English_Wikipedia"
text = "http://en.m.wikipedia.org"
org = (50, 50)
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.4
color = (255, 0, 0)
thickness = 1
image = cv2.putText(frame, text, org, font, fontScale, color, thickness, cv2.LINE_AA)

cv2.imshow('frame', image)

cv2.waitKey(0)