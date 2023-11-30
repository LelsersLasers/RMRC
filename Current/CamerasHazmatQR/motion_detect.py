import cv2
import numpy as np


def motion_detect_and_draw(frame_copy, avg, frame_dst, min_area, threshold_lower):
	contours = motion_detect(frame_copy, avg, threshold_lower)

	for cnt in contours:
		if cv2.contourArea(cnt) >= min_area:
			cv2.drawContours(frame_dst, [cnt], -1, (255, 0, 0), 3)
			x, y, w, h = cv2.boundingRect(cnt)
			cv2.rectangle(frame_dst, (x, y), (x + w, y + h), (0, 225, 0), 4)


def motion_detect(frame, avg, threshold_lower):
	frame = frame.astype("float")
	frame = cv2.GaussianBlur(frame, (21, 21), 0)
	
	frame_diff = cv2.absdiff(avg, frame)
	_, thresh = cv2.threshold(frame_diff, threshold_lower, 255, cv2.THRESH_BINARY)
	cv2.dilate(thresh, None, iterations=2)

	thresh_grayscale = cv2.cvtColor(np.float32(thresh), cv2.COLOR_BGR2GRAY)
	contours, _ = cv2.findContours(thresh_grayscale.astype("uint8"), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	return contours