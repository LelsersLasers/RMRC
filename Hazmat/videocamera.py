cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"

import threading
import cv2
import time

class CustomThread(threading.Thread):
    def __init__(self, args, start_fn):
        threading.Thread.__init__(self)
        self.args = args
        self.start_fn = start_fn

    def run(self):
        self.start_fn(self.args)
            
mut_state = {
    'frame': None,
    'quit': False
}

def screenshot(ms):
    if ms['frame'] is not None:
        print("screenshot")
        output_path = "screencapture.jpg"
        cv2.imwrite(output_path, ms['frame'])
        print(f"Saved the final image as {output_path}")
            
        return ms['frame']
    
def camera_feed(ms):
    cap = cv2.VideoCapture(cap_args)

    if not cap.isOpened():
        print("Can't open camera.")
        quit()
    
    time.sleep(1)

    while True:
        ret, frame = cap.read()
        if not ret or frame is None or ms['quit']:
            print("Exiting ...")
            break

        cv2.imshow("Camera feed", frame)

        if (cv2.waitKey(1) & 0xFF) == ord('g'):
            ms['frame'] = frame
            print("Got frame")

def start_feed():
    ct_camera_feed = CustomThread(mut_state, camera_feed)
    ct_camera_feed.start()

def end_feed():
    mut_state['quit'] = True
