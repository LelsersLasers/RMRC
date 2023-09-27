cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"

import cv2
import time

SAVE_KEY = 'g'

print(f"Press '{SAVE_KEY}' to save the current frame.")
print("Press 'q' to quit.")



cap = cv2.VideoCapture(cap_args)

if not cap.isOpened():
	print("Can't open camera.")
	quit()

time.sleep(1)

while True:
	ret, frame = cap.read()
	if not ret or frame is None:
		print("Exiting ...")
		break

	cv2.imshow("Camera feed", frame)

	if SAVE_KEY is not None and (cv2.waitKey(1) & 0xFF) == ord(SAVE_KEY):
		output_path = "screencapture.jpg"
		cv2.imwrite(output_path, frame)
		print(f"Saved the image as {output_path}")

	if (cv2.waitKey(1) & 0xFF) == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

