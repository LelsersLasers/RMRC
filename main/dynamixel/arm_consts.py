MAX_POSITION = 4096


ARM_REST_POSES = {
    "j1": 1024,
    "j2": 3250,
    "j3": 1200,
    "j4": 2048,
}

ARM_JOINT_OFFSETS = { # ARM_JOINT_OFFSETS[joint] = reader_pos - jetson_pos
    "j1": 0,
    "j2": 1500,
    "j3": -870,
    "j4": 0,
}
