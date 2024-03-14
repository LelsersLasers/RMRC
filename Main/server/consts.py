STATE_FROM_MASTER = {
    "frames": {
		"webcam1": "",
		"detection": "",
		"webcam2": "",
		"ir": "",
    },
	
    "hazmats_found": [],
    "qr_found": [],

    # webcam1, hazmat, webcam2, ir, master, motor, backend
    # motor and backend are set in server /get route, rest are sent from master.main.thread
    "fpses": [-1, -1, -1, -1, -1, -1, -1],
    
    "stats": {
        "ram": 0,
        "swap": 0,
        "cpu": 0,
        "cpu_freq": 0,
        "gpu": -1,
    },

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
        "motion": False,
    },
    "clear": {
        "hazmat": 0,
        "qr": 0,
    },
    "invert": False,
    "motion_min_area": 500,
    "motion_threshold": 65,
    "motion_new_frame_weight": 0.4,
    "hazmat_levenshtein_thresh": 0.4,
    "hazmat_angle_change": 90,
}