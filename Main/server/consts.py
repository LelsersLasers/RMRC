STATE_FROM_MASTER = {
    "frame": "",
    "ns": 0,
    "w": 1,
    "h": 1,
    "hazmats_found": [],
    "qr_found": [],
    "fpses": [-1, -1, -1, -1, -1, -1, -1], # [-2] = motor_fps, [-1] = server /get fps
    "ram": 0,
    "cpu": 0,
    "gpu": -1,
    "angle": 0,
}
STATE_FROM_SELF = {
    "run": {
        "hazmat": False,
        "qr": False,
        "md": False,
    },
    "clear": {
        "hazmat": 0,
        "qr": 0,
    },
    "view_mode": 0,
    "motion_min_area": 500,
    "motion_threshold": 65,
    "motion_new_frame_weight": 0.4,
    "hazmat_levenshtein_thresh": 0.4,
    "hazmat_angle_change": 90,
}