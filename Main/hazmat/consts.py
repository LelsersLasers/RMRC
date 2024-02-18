import time

# FPS for when not running detection
HAZMAT_DRY_FPS = 15

# What master thread sends
STATE_FROM_MASTER = {
    "frame": None, # latest main camera frame
    "run_hazmat": False,
    "quit": False,
    "clear": 0, # increment to clear all found hazmat labels
    "hazmat_levenshtein_thresh": 0.4,
    "hazmat_angle_change": 90,
}
# What hazmat thread sends
STATE_FROM_SELF = {
    "hazmat_fps": HAZMAT_DRY_FPS,
    "hazmat_frame": None,
    "hazmats_found": [],
    "last_update": time.time(),
    "angle": 0,
}