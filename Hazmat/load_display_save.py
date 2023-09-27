#only imported just so this works in any Python update
from __future__ import print_function
#in charge of parsing our command line arguments
import argparse
#the OpenCV library and contains the image processing functions
import cv2

ap = argparse.ArgumentParser()
#"--image" = the path to our image
#parse the arguments and store them into a dictionary
ap.add_argument("-i", "--image", required = True,
help = "Path to the image")
args = vars(ap.parse_args())

#load the iamge path off the disk 
#returns a NumPy array representing the image
#NumPy array = information abt data, how to interpet/locate it, etc
image = cv2.imread(args["image"])
#examine dimenisons of image (not difficult bc we already have NumPy array
# with all the info needed; just single out those attributes)
print("width: {} pixels".format(image.shape[1]))
print("height: {} pixels".format(image.shape[0]))
print("channels:{}".format(image.shape[2]))

#display actual image on screen
#first aprameter is a string, aka "name" of window
#second parameter refers to image loaded off disk
cv2.imshow("Image", image)
#pauses execution of script until specific key is pressed
#parameter of 0 = any keypress will un-pause execution
cv2.waitKey(0)

#write image to file in JPG format
#first argument = path to file
#second argument = image we want to save
cv2.imwrite("newimage.jpg", image)