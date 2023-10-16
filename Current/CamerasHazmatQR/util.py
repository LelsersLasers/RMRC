import time
import signal
import queue
from multiprocessing import Queue

class ViewMode:
    GRID = 0
    ZOOM = 1

    def __init__(self, start_mode = GRID):
        self.mode = start_mode
        self.zoom_on = -1

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
    
class FPS:
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
        self.q1.put(item)
    def put_q2(self, item):
        self.q2.put(item)

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

class Rect:
    def __init__(self, r):
        self.x = r[0]
        self.y = r[1]
        self.w = r[2]
        self.h = r[3]

    def __eq__(self, other):
        # True if self and other overlap
        return not (self.x + self.w <= other.x or other.x + other.w <= self.x or self.y + self.h <= other.y or other.y + other.h <= self.y)

class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
    def exit_gracefully(self, *args):
        self.kill_now = True