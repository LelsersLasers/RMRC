MAX_POSITION = 4096


ARM_REST_POSES = {
    "j1": 1024,
    "j2": 3250,
    "j3": 1200,
}

ARM_JOINT_OFFSETS = { # ARM_JOINT_OFFSETS[joint] = reader_pos - jetson_pos
    "j1": 0,
    "j2": 1670,
    "j3": -870,
}
