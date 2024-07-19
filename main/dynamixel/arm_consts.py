MAX_POSITION = 4096


ARM_REST_POSES = {
    "j1": 1024,
    "j2": 3500,
    "j3": 1300,
    "j4": None, # Whatever the starting position is, it is considered fully closed
}

ARM_JOINT_OFFSETS = { # ARM_JOINT_OFFSETS[joint] = reader_pos - jetson_pos
    "j1": 0,
    "j2": 900,
    "j3": -870,
    "j4": 0,
}

# J4_MIN = -800 # open
# J4_MAX = 725 # close
J4_RANGE = 600 # Note: lower = open
J4_CLOSE = 200