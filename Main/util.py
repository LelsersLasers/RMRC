import signal
import queue
import os
import time
import multiprocessing


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
        self.q1 = multiprocessing.Queue()
        self.q2 = multiprocessing.Queue()

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
def create_thread(target, args, name):
    process_name = f"{name}_process"
    thread = multiprocessing.Process(target=target, args=args, name=process_name)
    thread.daemon = True
    thread.start()
    print(f"{name} process pid: {thread.pid}")
    return thread
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