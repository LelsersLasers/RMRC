import cv2
import time

def screenshot():
    cap = cv2.VideoCapture("v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Can't open camera.")
        quit()
    else:
        print("Opened camera.")

    print("Press <Enter> when you are ready.")
    input()
    print("Click 'q' when satisfied with the image.")

    key = ""
    while key != ord("q"):
        _, frame = cap.read()
        cv2.imshow("image", frame)
        key = cv2.waitKey(1)

    print("All done gathering images.")
    cv2.imshow("test", frame)

    output_path = "new_image.jpg"  # Specify the desired output path and filename
    cv2.imwrite(output_path, frame)
    print(f"Saved the final image as {output_path}")
    return frame

if __name__=="__main__":
    screenshot()
