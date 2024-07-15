import time


CAP_ARGS = {
    "webcam1": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! video/x-raw,width=320,height=240,framerate=30/1 ! videoconvert ! appsink",
    "webcam2": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0 ! video/x-raw,width=320,height=240,framerate=30/1 ! videoconvert ! appsink",
    "ir":      "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__0055001f-500a-564e-3536-203400000000-video-index0 ! videoconvert ! appsink",
    # "arm":     "v4l2src device=/dev/v4l/by-id/usb-16MP_Camera_Mamufacture_16MP_USB_Camera_2022050701-video-index0 ! video/x-raw,format=YUY2,width=800,height=600,framerate=15/1 ! videoconvert ! appsink",
    "arm" :    "v4l2src device=/dev/v4l/by-id/usb-Image_Processor_USB_2.0_PC_Cam-video-index0 ! video/x-raw,width=320,height=240,framerate=30/1 ! videoconvert ! appsink",
}


CAMERA_SIZE = (320, 240)


CAMERA_WAIT_AFTER_ACTIVE = 3
CAMERA_WAKEUP_TIME = 1.5
CAMERA_WAIT_AFTER_FAIL = 1.0
CAMERA_NONE_GREY = 50

DRY_FPS = 2

STATE_FROM_MASTER = {
    "active_keys": ["webcam1", "webcam2"]
}

STATE_FROM_SELF = {
    "frame": None,
    "fps": 30,
    "time": time.time(),
}
