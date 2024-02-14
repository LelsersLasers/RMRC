import cv2
import numpy as np
import scipy

img = cv2.imread("test.png")

angel = 90
center = tuple(np.array(img.shape[1::-1]) / 2)

matrix = cv2.getRotationMatrix2D(center, angel, 1)
undo_matrix = cv2.getRotationMatrix2D(center, -angel, 1)

rotated_image = cv2.warpAffine(img, matrix, img.shape[1::-1])
unrotated_image = cv2.warpAffine(rotated_image, undo_matrix, img.shape[1::-1])


h, w = img.shape[:2]
sci_r = scipy.ndimage.rotate(img, angel)
sci_unr = scipy.ndimage.rotate(sci_r, -angel)
h2, w2 = sci_unr.shape[:2]
x1 = int((w2 - w) / 2)
y1 = int((h2 - h) / 2)
sci_unr = sci_unr[y1:y1+h, x1:x1+w]

images = [img, rotated_image, unrotated_image, sci_r, sci_unr]
fnames = ["img", "imgr", "imgunr", "sci_r", "sci_unr"]
for i, f in zip(images, fnames):
	fname = f + ".png"
	print(f, i.shape)
	
	h, w = i.shape[:2]
	cnt = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.int32)
	i = cv2.drawContours(i, [cnt], -1, (0, 255, 0), 5)

	cv2.imwrite(fname, i)