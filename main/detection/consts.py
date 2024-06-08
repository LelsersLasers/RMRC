import time

# FPS for when not running detection
DRY_FPS = 2


STATE_FROM_SERVER = {
    "frame": None, # latest primary camera frame
    "average_frame": None,
    
    "run": {
        "hazmat": False,
        "qr": False,
        "motion": False,
    },
    "clear": { # increment to clear all found labels
        "hazmat": 0,
        "qr": 0,
    },
    
    "hazmat_levenshtein_thresh": 0.4,
    "hazmat_angle_change": 90,
    
    "motion_min_area": 500,
    "motion_threshold": 65,
    "motion_new_frame_weight": 0.4,
}
STATE_FROM_SELF = {
    "fps": DRY_FPS,
    "frame": None,
    "found": {
        "hazmat": [],
        "qr": [],
    },
    "last_update": time.time(),
    "time_bars": {
        "qr": -1,
        "motion": -1,
    },
    "angle": 0,
}