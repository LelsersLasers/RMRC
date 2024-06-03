# Print before imports for instant feedback
print("STARTING...\n")

import argparse
import util
import laptop.main
import laptop.ps4_controller


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-a", "--no-arm-rest-pos", required=False, help="don't move the arm to default rest position", action="store_true")
    ap.add_argument("-p", "--ps4", required=False, help="Enable the PS4 controller", action="store_true")
    ap.add_argument("-z", "--video-capture-zero", required=False, help="testing version", action="store_true")
    args = vars(ap.parse_args())

    no_arm_rest_pos = args["no_arm_rest_pos"]
    ps4 = args["ps4"]
    video_capture_zero = args["video_capture_zero"]
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    if ps4:
        ps4_thread = util.create_thread(laptop.ps4_controller.thread, (video_capture_zero,), "ps4_controller")

    laptop.main.thread(no_arm_rest_pos, video_capture_zero)
    # ------------------------------------------------------------------------ #

    print("\n\nExiting...\n\n")

    # ------------------------------------------------------------------------ #
    if ps4:
        util.close_thread(ps4_thread)
    # ------------------------------------------------------------------------ #

    print("Done.")