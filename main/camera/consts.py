import time


CAP_ARGS = {
    "webcam1": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! video/x-raw,width=320,height=240,framerate=30/1 ! videoconvert ! appsink",
    "webcam2": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0 ! video/x-raw,width=320,height=240,framerate=30/1 ! videoconvert ! appsink",
    "ir":      "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! appsink",
    "arm":     "nvarguscamerasrc sensor_id=0 ! 'video/x-raw(memory:NVMM),framerate=30/1,format=NV12,width=320,height=240' ! nvvidconv ! appsink",
}

# working terminal arm camera command:
# gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! 'video/x-raw(memory:NVMM),framerate=30/1,format=NV12' ! nvvidconv ! xvimagesink

# old camera command in file on xavier
# gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! nvvidconv ! video/x-raw,format=NV12 ! videoscale ! video/x-raw,width=320,height=240 ! videorate ! video/x-raw,framerate=30/1 ! videoconvert ! appsink`

CAMERA_SIZE = (320, 240)


CAMERA_WAIT_AFTER_ACTIVE = 3
CAMERA_WAKEUP_TIME = 1.5
CAMERA_WAIT_AFTER_FAIL = 1.0
CAMERA_NONE_GREY = 50

DRY_FPS = 2

CAMERA_MODE_TO_ACTIVE_KEYS = { # CAMERA_MODE_TO_ACTIVE_KEYS = [base_key, alt_key, "ir"]
    "1": ["webcam1", "webcam2", "_ir"],
    "2": ["webcam1", "arm",     "_ir"],
    "3": ["arm",     "webcam1", "_ir"],
    "4": ["webcam2", "arm",     "_ir"],
}


STATE_FROM_MASTER = {
    "active_keys": CAMERA_MODE_TO_ACTIVE_KEYS["1"],
}

STATE_FROM_SELF = {
    "frame": None,
    "fps": 30,
    "time": time.time(),
}
