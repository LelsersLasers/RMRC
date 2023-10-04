# webcam1_cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"
# webcam2_cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"
# ir_cap_args = "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! appsink"

cap_args = {
    "webcam1": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "webcam2": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "ir": "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! appsink",
}


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

    caps = {}
    for key, value in cap_args.items():
        caps[key] = cv2.VideoCapture(value, cv2.CAP_GSTREAMER)

    for key, cap in caps.items():
        if not cap.isOpened():
            raise RuntimeError(
                f"Can't open camera {key}. Are the cap_args set right? Is the camera plugged in?"
            )

    time.sleep(1)

    print(f"\nPress '{HAZMAT_KEY}' to run hazmat. (Note: hazmat is blocking)")
    print("Press 'q' to close.")

    hazmat_frame = None

    while True:
        # webcam1_ret, webcam1_frame = webcam1_cap.read()
        # webcam2_ret, webcam2_frame = webcam2_cap.read()
        # ir_ret, ir_frame = ir_cap.read()

        # if False in [webcam1_ret, webcam2_ret, ir_ret] or None in [webcam1_frame, webcam2_frame, ir_frame]:
        #     print("Exiting ...")
        #     break

        frames = {}
        for key, cap in caps.items():
            ret, frame = cap.read()

            if not ret or frame is None:
                print("Exiting ...")
                return

            frames[key] = frame

        webcam1_shape = frames["webcam1"].shape
        ir_frame = cv2.resize(frames["ir"], (webcam1_shape[1], webcam1_shape[0]))


        key = cv2.waitKey(1) & 0xFF

        if HAZMAT_KEY is not None and key == ord(HAZMAT_KEY):
            cv2.imwrite(FILENAME, frames["webcam1"])
            print(f"Saved the image as {FILENAME}. Starting running hazmat detection...")
            hazmat_frame = hazmat.hazmat_main(frames["webcam1"], show_window=False)

        hazmat_frame_to_display = hazmat_frame if hazmat_frame is not None else np.zeros_like(frames["webcam1"])

        top_combined = cv2.hconcat([frames["webcam1"], hazmat_frame_to_display])
        bottom_combined = cv2.hconcat([frames["webcam2"], ir_frame])

        combined = cv2.vconcat([top_combined, bottom_combined])

        cv2.imshow("Camera feed", combined)

        if key == ord("q"):
            break

    for cap in caps.values():
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()