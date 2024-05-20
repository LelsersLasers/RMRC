BASE_TEST_URL = "http://127.0.0.1:5001"
BASE_URL      = "http://192.168.1.6:5001"

ARM_TEST_URL = BASE_TEST_URL + "/arm/" # + j1 + "/" + j2 + "/" + j3 + "/" + fps + "/" + time
ARM_URL      = BASE_URL      + "/arm/" # + j1 + "/" + j2 + "/" + j3 + "/" + fps + "/" + time

READER_TEST_FPS = 10
READER_TEST_DRIFT_RESET = READER_TEST_FPS * 5

GET_FAIL_WAIT = 0.5