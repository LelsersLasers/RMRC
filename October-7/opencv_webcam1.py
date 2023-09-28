cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"

import cv2
import time

SAVE_KEY = 'g'
FILENAME = "screencapture.jpg"

print(f"Press '{SAVE_KEY}' to save the current frame.")
print("Press 'q' to close.")


def screenshot():
	img = cv2.imread(FILENAME)
	
	if img is None:
		raise FileNotFoundError(f"Could not find/load {FILENAME}. Is the video capture running? Did you use {SAVE_KEY} to save the image?}")
	
	return img


def main():
	cap = cv2.VideoCapture(cap_args)

	if not cap.isOpened():
		raise RuntimeError("Can't open camera. Are the cap_args set right? Is the camera plugged in?")

	time.sleep(1)

	while True:
		ret, frame = cap.read()
		if not ret or frame is None:
			print("Exiting ...")
			break

		cv2.imshow("Camera feed", frame)

		if SAVE_KEY is not None and (cv2.waitKey(1) & 0xFF) == ord(SAVE_KEY):
			cv2.imwrite(FILENAME, frame)
			print(f"Saved the image as {FILENAME}")

		if (cv2.waitKey(1) & 0xFF) == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()

