import time

PORT = 5001

STATE_FROM_SELF = {
    "cycles": {
        "j1": 0,
        "j2": 0,
        "j3": 0,
    },
    "arm_target_positions": {
        "j1": 1024,
        "j2": 3250,
        "j3": 1200,
    },
    "arm_reader_fps": 20,
    "time": time.time(),
}

STATE_FROM_MOTORS = {
    "arm_active": False,
}