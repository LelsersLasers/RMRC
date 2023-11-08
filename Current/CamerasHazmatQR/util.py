import time
import signal
import queue
import os

import numpy as np
import cv2
import scipy

from multiprocessing import Queue

# ---------------------------------------------------------------------------- #
class ViewMode:
    GRID = 0
    ZOOM = 1

    def __init__(self, start_mode = GRID):
        self.mode = start_mode
        self.zoom_on = -1
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class Toggler:
    def __init__(self, start_state=False):
        self.state = start_state

    def toggle(self):
        self.state = not self.state

    def get(self):
        return self.state
    
    def __bool__(self):
        return self.state
    
    def __str__(self):
        return str(self.state)
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class ToggleKey:
    def __init__(self, default = False):
        self.was_down = default

    def down(self, condition):
        if not self.was_down and condition:
            self.was_down = True
            return True
        elif not condition:
            self.was_down = False
        return False
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class FPSController:
    def __init__(self, start_delta = 1/30):
        self.t0 = time.time()
        self.t1 = time.time()
        self.delta = start_delta

    def update(self):
        self.t1 = time.time()
        self.delta = self.t1 - self.t0
        self.t0 = self.t1

        return self.delta
    
    def fps(self):
        if self.delta == 0:
            return -1
        else:
            return 1 / self.delta
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def close_thread(t):
    t.join(1)

    t.terminate()

    t.join(1)

    try:    
        t.close()
    except:
        pass

def last_from_queue(q, last_value):
	value = last_value

	while True:
		try:
			value = q.get_nowait()
		except queue.Empty:
			break

	return value

class DoubleQueue:
    def __init__(self):
        self.q1 = Queue()
        self.q2 = Queue()

    def put_q1(self, item):
        self.q1.put_nowait(item)
    def put_q2(self, item):
        self.q2.put_nowait(item)

    def last_q1(self, value):
        return last_from_queue(self.q1, value)
    def last_q2(self, value):
        return last_from_queue(self.q2, value)
    
    def close(self):
        self.q1.close()
        self.q2.close()

        # basically `allow_exit_without_flush`
        self.q1.cancel_join_thread()
        self.q2.cancel_join_thread()

class DoubleState:
    def __init__(self, s1, s2):
        self.s1 = s1
        self.s2 = s2

    def update_s1(self, dq):
        self.s1 = dq.last_q1(self.s1)
    def update_s2(self, dq):
        self.s2 = dq.last_q2(self.s2)

    def put_s1(self, dq):
        dq.put_q1(self.s1)
    def put_s2(self, dq):
        dq.put_q2(self.s2)
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
            cv2.drawContours(mask1, [cnt], -1, 255, 50)
        
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
    

def unrotate_cnt(cnt_rotated, rotated, img_shape):
    img_h, img_w = img_shape[:2]

    mask = np.zeros(rotated.image.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [cnt_rotated], -1, 255, -1)
    
    mask = scipy.ndimage.rotate(mask, -rotated.angle)
    h, w = mask.shape[:2]
    x = int((w - img_w) / 2)
    y = int((h - img_h) / 2)
    mask = mask[y:y+img_h, x:x+img_w]

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours[0]

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
    
class LevenshteinResult:
    def __init__(self, detection_result, closest, ratio):
        self.detection_result = detection_result
        self.closest = closest
        self.ratio = ratio
        
        display_confidence = detection_result.confidence * 100
        self.string = f'{closest} (\'{detection_result.text}\', {display_confidence:.0f}%, {ratio:.2f})' 
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
    def exit_gracefully(self, *args):
        self.kill_now = True
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def remove_dups(list, comp):
    new_list = []
    for item in list:
        if comp(item) not in [comp(x) for x in new_list]:
            new_list.append(item)
    return new_list
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def read_last_line(f):
    # with open('filename.txt', 'rb') as f:
    try:  # catch OSError in case of a one line file 
        f.seek(-2, os.SEEK_END)
        while f.read(1) != b'\n':
            f.seek(-2, os.SEEK_CUR)
    except OSError:
        f.seek(0)
        
    last_line = f.readline().decode()
    return last_line
# ---------------------------------------------------------------------------- #