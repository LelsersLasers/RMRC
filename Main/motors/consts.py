import time

MAX_POWER_START = 330
MOTOR_MIN_WRITES = 1
MAX_WRITES = 3

MOTOR_SHUTOFF_TIME = 1.0

CLOSE_WAIT_TIME = 0.25

MOTOR_TEST_FPS = 10



STATE_FROM_SERVER = {
    "left": 0,
    "right": 0,
    "count": 0,
    "last_get": time.time(),
    "velocity_limit": {
		"value": MAX_POWER_START,
		"count": 0,
    },
    "motor_writes": MOTOR_MIN_WRITES,
    "write_every_frame": False,
}
STATE_FROM_SELF = {
    "motors": {
        "target": {
            "left": 0,
            "right": 0,
        },
        "current": {
            "left": 0,
            "right": 0,
        }
    },
    "motor_fps": 20,
}

STATE_FROM_MASTER = {
    "quit": False,
}