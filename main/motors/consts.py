import time

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
        "value": 330,
        "count": 0,
    },
    "motor_writes": 1,
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