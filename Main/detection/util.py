import cv2
import scipy
import numpy as np


# ---------------------------------------------------------------------------- #
class Rotated:
    def __init__(self, image, angle):
        self.image = image
        self.angle = angle

    @classmethod
    def from_image_and_angle(cls, image, angle):
        rotated_image = scipy.ndimage.rotate(image, angle)
        return cls(rotated_image, angle)
    
    def unrotate_cnt(self, cnt_rotated, img_shape):
        img_h, img_w = img_shape[:2]

        mask = np.zeros(self.image.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [cnt_rotated], -1, 255, -1)
        
        mask = scipy.ndimage.rotate(mask, -self.angle)
        h, w = mask.shape[:2]
        x = int((w - img_w) / 2)
        y = int((h - img_h) / 2)
        mask = mask[y:y+img_h, x:x+img_w]

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours[0]
# ---------------------------------------------------------------------------- #

# ---------------------------------------------------------------------------- #
class CNT:
    def __init__(self, cnt, frame_shape, expand):
        self.cnt = cnt
        self.frame_shape = frame_shape

        self.mask_shape = (frame_shape[0], frame_shape[1], 1)
        mask1 = np.zeros(self.mask_shape, np.uint8)

        cv2.drawContours(mask1, [cnt], -1, 255, -1)
        
        if expand:
            cv2.drawContours(mask1, [cnt], -1, 255, 65)
        
        self.pixel_count = cv2.countNonZero(mask1)

    def draw_both(self, other):
        mask = np.zeros(self.mask_shape, np.uint8)
        cv2.drawContours(mask, [self.cnt], -1, 255, -1)
        cv2.drawContours(mask, [other.cnt], -1, 255, -1)
        return mask
    
    def combine(self, other):
        mask = self.draw_both(other)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return CNT(contours[0], self.frame_shape, False)

    def __eq__(self, other):
        # True if self and other overlap
        mask = self.draw_both(other)
        px_count = cv2.countNonZero(mask)

        return px_count < self.pixel_count + other.pixel_count
# ---------------------------------------------------------------------------- #

# ---------------------------------------------------------------------------- #
class DetectionResult:
    def __init__(self, cnt, text, confidence):
        self.cnt = cnt
        self.text = text.strip()
        self.confidence = confidence

    def overlaps(self, other):
        return self.cnt == other.cnt

    def combine(self, other):
        cnt = self.cnt.combine(other.cnt)
        text = f'{self.text} {other.text}'
        confidence = (self.confidence + other.confidence) / 2

        return DetectionResult(cnt, text, confidence)
    
def combine_nearby(detection_results):
    new_detection_results = []
    for i, detection_result_a in enumerate(detection_results):
        for j, detection_result_b in enumerate(detection_results):
            if i == j:
                continue

            if detection_result_a.overlaps(detection_result_b):
                new_detection_result = detection_result_a.combine(detection_result_b)
                new_detection_results.append(new_detection_result)
    
    for new_detection_result in new_detection_results:
        detection_results.append(new_detection_result)
# ---------------------------------------------------------------------------- #

# ---------------------------------------------------------------------------- #
class LevenshteinResult:
    def __init__(self, detection_result, closest, ratio):
        self.detection_result = detection_result
        self.closest = closest
        self.ratio = ratio
        
        display_confidence = detection_result.confidence * 100
        self.string = f'{closest} (\'{detection_result.text}\', {display_confidence:.0f}%, {ratio:.2f})' 

def remove_dups(list, comp):
    new_list = []
    for item in list:
        # x in xs uses __eq__ to compare
        if comp(item) not in [comp(x) for x in new_list]:
            new_list.append(item)
    return new_list
# ---------------------------------------------------------------------------- #