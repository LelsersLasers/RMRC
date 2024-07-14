MAX_POSITION = 4096


ARM_REST_POSES = {
    "j1": 1024,
    "j2": 3640,
    "j3": 1830,
    "j4": 4800,
}

ARM_JOINT_OFFSETS = { # ARM_JOINT_OFFSETS[joint] = reader_pos - jetson_pos
    "j1": 0,
    "j2": 1500,
    "j3": -870,
    "j4": 0,
}

J4_MIN = 3270 # open
J4_MAX = 4800 # close
