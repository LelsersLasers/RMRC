import argparse
import laptop.main


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-z", "--video-capture-zero", required=False, help="testing version", action="store_true")
    args = vars(ap.parse_args())

    video_capture_zero = args["video_capture_zero"]

    laptop.main.thread(video_capture_zero)

    print("Exiting...")