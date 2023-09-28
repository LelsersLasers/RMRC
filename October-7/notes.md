# For getting hazmat a camera feed

## CURRENT ISSUES

### OpenCV on MacBook

- Can not use `import cv2` on macbook which makes life more difficult
- As of now, invalidates any solutions that run hazmat code on MacBook

## Hazmat/screencapture.py

- Screenshots top left of screen
- Must put the right camera feed from gstreamer in the right spot
- Runs hazmat code on MacBook

## Hazmat/videocamera.py

- Multithreading to run camera feed in openCV without blocking hazmat code
- Hazmat code needs to start and stop camera feed with `start_feed()` and `end_feed()`
- Runs hazmat code on Jetson

## October-7/opencv_webcam1.py

- Runs camera feed in openCV and saves frame to disk
- Hazmat code should be adjusted to use file from disk instead of returned from from `screenshot()`
- Runs hazmat code on Jetson

## Literally screenshot

- Take a screenshot of the gstreamer window and give it to the hazmat code
- Runs hazmat code on MacBook