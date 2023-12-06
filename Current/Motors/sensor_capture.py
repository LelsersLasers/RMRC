import cv2

GSTREAMER_PIPELINE = '''nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM), 
width=1280, height=720, format=(string)NV12, framerate=21/1 ! 
nvvidconv flip-method=2 ! video/x-raw, width=1280, height=720, 
format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'''

# Assuming front camera is at /dev/video1
cam = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)

ret, frame = cam.read()
cv2.imwrite("sensor_2.jpg", frame)

cam.release()
cv2.destroyAllWindows()
