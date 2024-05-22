import time

PORT = 5001

STATE_FROM_SELF = {
    "arm_target_positions": {
        "j1": 3072,
        "j2": 1024,
        "j3": 800,
    },
    "arm_reader_fps": 20,
    "time": time.time(),
}

STATE_FROM_MOTORS = {
    "arm_active": False,
}