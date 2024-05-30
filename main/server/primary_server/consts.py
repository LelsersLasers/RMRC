PORT = 5000

STATE_FROM_MASTER = {
    "frames": {
        "webcam1": "",
        "arm": "",
        "webcam2": "",
        "detection_ir": "",
    },
    
    "hazmats_found": [],
    "qr_found": [],

    # webcam1, detections, webcam2, ir, master, motor, armreader, backend
    # "webcam1", "webcam2", "arm", "detection", "ir", "master", "motor", "armreader", "backend"
    # motor, armreader, backend are set in server /get route, rest are sent from master.main.thread
    "fpses": [-1, -1, -1, -1, -1, -1, -1, -1, -1],
    
    "stats": {
        "ram": 0,
        "swap": 0,
        "cpu": 0,
        "gpu": -1,
    },

    "temps": {
        "cpu": -1,
        "gpu": -1,
        "aux": -1,
    },

    "angle": 0,
    "time_bars": {
        "hazmat": 0,
        "qr": 0,
        "motion": 0,
    }
}
STATE_FROM_SELF = {
    "run": {
        "hazmat": False,
        "qr": False,
        "motion": False,
    },
    "clear": {
        "hazmat": 0,
        "qr": 0,
    },
    "invert": False,
    "show_detections": False,
    "motion_min_area": 500,
    "motion_threshold": 65,
    "motion_new_frame_weight": 0.4,
    "hazmat_levenshtein_thresh": 0.4,
    "hazmat_angle_change": 90,
    "master_fps": 200,
}