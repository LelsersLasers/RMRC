STATE_FROM_MASTER = {
    "frame": "",
    "w": 1,
    "h": 1,
    "hazmats_found": [],
    "qr_found": [],

    # webcam1, hazmat, webcam2, ir, master, motor, backend
    # motor and backend are set in server /get route, rest are sent from master.main.thread
    "fpses": [-1, -1, -1, -1, -1, -1, -1],
    
    "ram": 0,
    "cpu": 0,
    "gpu": -1,
    "update_combined_ratio": 0,

    "angle": 0,
    "timebars": {
        "hazmat": 0,
        "qr": 0,
        "motion": 0,
    }
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
    "view_mode": {
        "value": 0,
        "count": 0,
    },
    "invert": False,
    "motion_min_area": 500,
    "motion_threshold": 65,
    "motion_new_frame_weight": 0.4,
    "hazmat_levenshtein_thresh": 0.4,
    "hazmat_angle_change": 90,
}