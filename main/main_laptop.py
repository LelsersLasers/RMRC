import argparse
import laptop.main


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-a", "--no-arm-rest-pos", required=False, help="don't move the arm to default rest position", action="store_true")
    ap.add_argument("-z", "--video-capture-zero", required=False, help="testing version", action="store_true")
    args = vars(ap.parse_args())

    no_arm_rest_pos = args["no_arm_rest_pos"]
    video_capture_zero = args["video_capture_zero"]

    laptop.main.thread(no_arm_rest_pos, video_capture_zero)

    print("Exiting...")