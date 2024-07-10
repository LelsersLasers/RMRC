import time

PORT = 5001

STATE_FROM_SELF = {
    "cycles": {
        "j1": 0,
        "j2": 0,
        "j3": 0,
        "j4": 0,
    },
    "arm_target_positions": {
        "j1": 1024,
        "j2": 3250,
        "j3": 1200,
        "j4": 2048,
    },
    "arm_reader_fps": 20,
    "arm_time": 0,

    "left": 0,
    "right": 0,
    "count": 0,

    "power_percent": 0.6,
}

STATE_FROM_MOTORS = {
    "arm_active": False,
    "invert": False,
	"high_send_rate": 20,
}