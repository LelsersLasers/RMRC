import time
import signal
import numpy as np
import cv2
import queue
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
    def __init__(self, cnt, frame_shape):
        self.cnt = cnt
        self.frame_shape = frame_shape

        self.mask_shape = (frame_shape[0], frame_shape[1], 1)
        mask1 = np.zeros(self.mask_shape, np.uint8)
        cv2.drawContours(mask1, [cnt], -1, 255, -1)
        self.pixel_count = cv2.countNonZero(mask1)

    def __eq__(self, other):
        # True if self and other overlap
        mask = np.zeros(self.mask_shape, np.uint8)
        cv2.drawContours(mask, [self.cnt], -1, 255, -1)
        cv2.drawContours(mask, [other.cnt], -1, 255, -1)
        px_count = cv2.countNonZero(mask)

        return px_count < self.pixel_count + other.pixel_count
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

def removeSpecialCharacter(s):
    t = ""
    for i in s:
        if i >= 'A' and i <= 'Z' or i == " ":
            t += i
    return t
# ---------------------------------------------------------------------------- #