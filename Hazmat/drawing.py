import numpy as np
import cv2

#intializing the image
#300 by 300 pixel image with 3 channels (red, green and blue)
#np.zeros fills every element with an intial value of zero
canvas = np.zeros((300, 300, 3), dtype = "uint8")

green = (0, 255, 0)
cv2.line(canvas, (0,0), (300,300), green)
cv2.imshow("Canvas", canvas)
cv2.waitKey(0)

red = (0, 0, 255)
cv2.line(canvas, (300, 0), (0, 300), red, 3)
cv2.imshow("Canvas", canvas)
cv2.waitKey(0)

cv2.rectangle(canvas, (10,10), (60,60), green)
cv2.imshow("Canvas", canvas)
cv2.waitKey(0)

blue = (255, 0, 0)
cv2.rectangle(canvas, (200,50), (225, 125), blue, -1)
cv2.imshow("Canvas", canvas)
cv2.waitKey(0) 

#new blank canvas
canvas = np.zeros((300, 300, 3), dtype="uint8")
(centerX, centerY) = (canvas.shape[1] // 2, canvas.shape[0] //2)
white=(255, 255, 255)

for r in range (0, 175, 25):
    cv2.circle(canvas, (centerX, centerY), r, white)

cv2.imshow("Canvas", canvas)
cv2.waitKey(0)

for i in range(0,7):
    radius = np.random.randint(5, high=200)
    #gets 3 random integers
    color = np.random.randint(0, high = 256, size = (3,)).tolist()
    #gets 2 random integers
    pt = np.random.randint(0, high = 200, size = (2,))
    cv2.circle(canvas, tuple(pt), radius, color, -1)

cv2.imshow("Canvas", canvas)
cv2.waitKey(0)