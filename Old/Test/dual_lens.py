import cv2
import numpy as np

OVERLAP_WIDTH = 100

def combine_linear_gradient(left, right):
	# Creating a smooth gradient from 0 to 1 in the overlap region
	left_mask = np.repeat(np.tile(np.linspace(1, 0, left.shape[1]), (left.shape[0], 1))[:, :, np.newaxis], 3, axis=2)
	right_mask = np.repeat(np.tile(np.linspace(0, 1, right.shape[1]), (right.shape[0], 1))[:, :, np.newaxis], 3, axis=2)

	blended = cv2.addWeighted(left, left_mask, right, right_mask, 0.0)
	return blended