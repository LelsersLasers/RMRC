import time

MOTOR_SHUTOFF_TIME = 1.0
MOTOR_TEST_FPS = 10
# ---------------------------------------------------------------------------- #

# ---------------------------------------------------------------------------- #
STATE_FROM_SERVER = {
    "left": 0,
    "right": 0,
    "count": 0,
    "last_get": time.time(),
    "velocity_limit": {
        "value": 330,
        "count": 0,
    },
    "arm_active": False,
    "motor_writes": 1,
    "write_every_frame": False,
    "arm_target_positions": {
        "j1": 3072,
        "j2": 1024,
        "j3": 800,
    },
    "arm_reader_fps": 20,
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
	"arm": {
		"target": {
            "j1": 0,
            "j2": 0,
            "j3": 0,
        },
        "current": {
            "j1": 0,
            "j2": 0,
            "j3": 0,
        },
        "active": False,
    },
    "arm_reader_fps": 20,
    "motor_fps": 20,
}