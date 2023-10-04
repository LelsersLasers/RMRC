webcam1_cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"
webcam2_cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! xvimagesink"
ir_cap_args = "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! xvimagesink"


import cv2
import time
import hazmat
import numpy as np
# import subprocess

HAZMAT_KEY = "g"
FILENAME = "screencapture.jpg"


def screenshot():
    img = cv2.imread(FILENAME)

    if img is None:
        raise FileNotFoundError(
            f"Could not find/load {FILENAME}. Is the video capture running? Did you use {HAZMAT_KEY} to save the image?"
        )

    return img


def main():
    print("Starting camera feed...")
    webcam1_cap = cv2.VideoCapture(webcam1_cap_args, cv2.CAP_GSTREAMER)
    webcam2_cap = cv2.VideoCapture(webcam2_cap_args, cv2.CAP_GSTREAMER)
    ir_cap = cv2.VideoCapture(ir_cap_args, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        raise RuntimeError(
            "Can't open camera. Are the cap_args set right? Is the camera plugged in?"
        )

    time.sleep(1)

    print(f"\nPress '{HAZMAT_KEY}' to run hazmat. (Note: hazmat is blocking)")
    print("Press 'q' to close.")

    hazmat_frame = None

    while True:
        webcam1_ret, webcam1_frame = webcam1_cap.read()
        webcam2_ret, webcam2_frame = webcam2_cap.read()
        ir_ret, ir_frame = ir_cap.read()

        if False in [webcam1_ret, webcam2_ret, ir_ret] or None in [webcam1_frame, webcam2_frame, ir_frame]:
            print("Exiting ...")
            break

        webcam1_shape = webcam1_frame.shape
        ir_frame = cv2.resize(ir_frame, (webcam1_shape[1], webcam1_shape[0]))


        key = cv2.waitKey(1) & 0xFF

        if HAZMAT_KEY is not None and key == ord(HAZMAT_KEY):
            cv2.imwrite(FILENAME, webcam1_frame)
            print(f"Saved the image as {FILENAME}. Starting running hazmat detection...")
            hazmat_frame = hazmat.hazmat_main(webcam1_frame, False)

        hazmat_frame_to_display = hazmat_frame if hazmat_frame is not None else np.zeros_like(webcam1_frame)

        top_combined = cv2.hconcat([webcam1_frame, hazmat_frame_to_display])
        bottom_combined = cv2.hconcat([webcam2_frame, ir_frame])

        combined = cv2.vconcat([top_combined, bottom_combined])

        cv2.imshow("Camera feed", combined)

        if key == ord("q"):
            break

    webcam1_cap.release()
    webcam2_cap.release()
    ir_cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
