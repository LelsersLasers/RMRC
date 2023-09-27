# For getting hazmat a camera feed

## Hazmat/screencapture.py

- Screenshots top left of screen
- Must put the right camera feed from gstreamer in the right spot

## Hazmat/videocamera.py

- Multithreading to run camera feed in openCV without blocking hazmat code
- Hazmat code needs to start and stop camera feed with `start_feed()` and `end_feed()`

## October-7/opencv_webcam1.py

- Runs camera feed in openCV and saves frame to disk
- Hazmat code should be adjusted to use file from disk instead of returned from from `screenshot()`